#include "control.h"
#include <math.h>
#include "attitude.h"
#include "pwm.h"

/* ══════════════════════════════════════════════════════════════
 *  配置常量
 * ══════════════════════════════════════════════════════════════ */

#define THROTTLE_DEADBAND 2.0f
#define GIMBAL_LOCK_THRESH 0.95f /* sin(pitch) > 此值时禁用 yaw */
#define M_PI_F 3.14159265f
#define TO_DEG 114.5916f /* 180 / π */

/* ══════════════════════════════════════════════════════════════
 *  PID & 公开变量
 * ══════════════════════════════════════════════════════════════ */

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = 50.0f;

/* 临界区保护的共享变量 */
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;

/* ══════════════════════════════════════════════════════════════
 *  内部状态
 * ══════════════════════════════════════════════════════════════ */

static ControlMode_t curMode = CONTROL_MODE_DIRECT;
static FlightPhase_t curPhase = FLIGHT_PHASE_GROUNDED;
static volatile uint8_t isArmed = 0;
static int8_t rollPitchSign = 1;

static float baseHeight = 0.0f;
static float targetRoll = 0.0f;
static float targetPitch = 0.0f;
static float targetYaw = 0.0f;
static float targetHeight = 0.0f;
static float moveForward = 0.0f;
static float moveRight = 0.0f;

/* ══════════════════════════════════════════════════════════════
 *  内部工具
 * ══════════════════════════════════════════════════════════════ */

static float NormalizeAngle(float a) {
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}

static void ResetAllPIDs(void) {
    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

static void ResetAllTargets(void) {
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    targetYaw = 0.0f;
    targetHeight = baseHeight;
    moveForward = 0.0f;
    moveRight = 0.0f;
}

static void StopMotors(void) {
    __disable_irq();
    thrustOutput = 0.0f;
    rateSetRoll = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw = 0.0f;
    __enable_irq();

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

/* ══════════════════════════════════════════════════════════════
 *  外环（姿态 + 高度）— 由主循环以 200 Hz 调用
 * ══════════════════════════════════════════════════════════════ */

void ControlAttitude_Loop(void) {
    if (!isArmed) {
        StopMotors();
        return;
    }

    const float dt = 1.0f / (float)ATTITUDE_LOOP_HZ;

    /* ── 读取传感器 ──────────────────────────────── */
    sm_quat_t q;
    Attitude_GetQuat(q);

    float curRoll, curPitch, curYaw;
    Attitude_GetEuler(&curRoll, &curPitch, &curYaw);

    float curHeight;
    Attitude_GetAltitude(&curHeight);

    /* ── 降落状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_LANDING) {
        if (curHeight < baseHeight + 0.05f) {
            curPhase = FLIGHT_PHASE_GROUNDED;
            Control_Disarm();
            return;
        }
    }

    /* ── 起飞状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_TAKING_OFF) {
        if (fabsf(curHeight - targetHeight) < 0.10f) {
            curPhase = FLIGHT_PHASE_IN_FLIGHT;
        }
    }

    /* ── 高度环 ──────────────────────────────────── */
    if (curMode >= CONTROL_MODE_ALTITUDE) {
        thrustOutput = baseThrottle + PID_Update(&pidHeight, targetHeight, curHeight, dt);
        thrustOutput = fmaxf(0.0f, fminf(thrustOutput, 100.0f));
    }

    /* ── DIRECT 模式：不输出角度环 ────────────────── */
    if (curMode == CONTROL_MODE_DIRECT) {
        __disable_irq();
        rateSetRoll = 0.0f;
        rateSetPitch = 0.0f;
        rateSetYaw = 0.0f;
        __enable_irq();
        return;
    }

    /* ── 姿态指令 ────────────────────────────────── */
    float effRoll = targetRoll;
    float effPitch = targetPitch;

    if (curMode >= CONTROL_MODE_ALTITUDE) {
        effPitch -= moveForward * 25.0f;
        effRoll += moveRight * 25.0f;
    }

    /* ── 四元数误差计算 ──────────────────────────── */
    float curYawAngle = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                               1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    float yawErrDeg = NormalizeAngle(targetYaw - curYawAngle * 180.0f / M_PI_F);
    float targetYawRad = curYawAngle + yawErrDeg * M_PI_F / 180.0f;
    
    float halfYaw = targetYawRad * 0.5f;
    float halfPitch = effPitch * (M_PI_F / 180.0f) * 0.5f;
    float halfRoll = effRoll * (M_PI_F / 180.0f) * 0.5f;

    float cy = cosf(halfYaw);
    float sy = sinf(halfYaw);
    float cp = cosf(halfPitch);
    float sp = sinf(halfPitch);
    float cr = cosf(halfRoll);
    float sr = sinf(halfRoll);

    float tw = cr * cp * cy + sr * sp * sy;
    float tx = sr * cp * cy - cr * sp * sy;
    float ty = cr * sp * cy + sr * cp * sy;
    float tz = cr * cp * sy - sr * sp * cy;

    float ew = q[0] * tw + q[1] * tx + q[2] * ty + q[3] * tz;
    float ex = -q[1] * tw + q[0] * tx + q[3] * ty - q[2] * tz;
    float ey = -q[2] * tw + q[3] * tx + q[0] * ty + q[1] * tz;
    float ez = -q[3] * tw + q[2] * tx + q[1] * ty + q[0] * tz;

    /* 最短路径 + 符号修正 */
    float sign = (ew >= 0.0f) ? -1.0f : 1.0f;

    float errRoll = sign * ex;
    float errPitch = sign * ey;
    float errYaw = sign * ez;

    /* Gimbal Lock 保护 */
    float sinPitch = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    if (fabsf(sinPitch) > GIMBAL_LOCK_THRESH) {
        errYaw = 0.0f;
    }

    /* ── 角度环 PID ──────────────────────────────── */
    float newRateSetRoll = PID_Update(&pidRoll, 0.0f, errRoll * TO_DEG, dt);
    float newRateSetPitch = PID_Update(&pidPitch, 0.0f, errPitch * TO_DEG, dt);
    float newRateSetYaw = PID_Update(&pidYaw, 0.0f, errYaw * TO_DEG, dt);

    /* 原子写入共享变量 */
    __disable_irq();
    rateSetRoll = newRateSetRoll;
    rateSetPitch = newRateSetPitch;
    rateSetYaw = newRateSetYaw;
    __enable_irq();
}

/* ══════════════════════════════════════════════════════════════
 *  内环（速率）— 由主循环或中断调用
 * ══════════════════════════════════════════════════════════════ */

void ControlMotor_Loop(void) {
    if (!isArmed) {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
        TIM3->CCR3 = 0;
        TIM3->CCR4 = 0;
        return;
    }

    const float dt = 1.0f / (float)RATE_LOOP_HZ;

    /* 原子读取共享变量 */
    float localRateSetRoll, localRateSetPitch, localRateSetYaw;
    __disable_irq();
    localRateSetRoll = rateSetRoll;
    localRateSetPitch = rateSetPitch;
    localRateSetYaw = rateSetYaw;
    __enable_irq();

    sm_vec3_t gyro;
    Attitude_GetGyro(gyro);
    float gx = (float)rollPitchSign * gyro[0];
    float gy = (float)rollPitchSign * gyro[1];
    float gz = (float)rollPitchSign * gyro[2];

    float rollCtrl = PID_Update(&pidRateRoll, localRateSetRoll, gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, localRateSetPitch, gy, dt);
    float yawCtrl = PID_Update(&pidRateYaw, localRateSetYaw, gz, dt);

    float throttle = thrustOutput;

    /* 电机混控 */
    float m[4];
    m[0] = throttle - pitchCtrl - rollCtrl + yawCtrl;
    m[1] = throttle + pitchCtrl - rollCtrl - yawCtrl;
    m[2] = throttle - pitchCtrl + rollCtrl - yawCtrl;
    m[3] = throttle + pitchCtrl + rollCtrl + yawCtrl;

    TIM3->CCR1 = PWM_Map_Percent(m[0]);
    TIM3->CCR2 = PWM_Map_Percent(m[1]);
    TIM3->CCR3 = PWM_Map_Percent(m[2]);
    TIM3->CCR4 = PWM_Map_Percent(m[3]);
}

/* ══════════════════════════════════════════════════════════════
 *  初始化
 * ══════════════════════════════════════════════════════════════ */

void Control_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TB;
    uint32_t timer_clk = SystemCoreClock;
    uint16_t presc = (uint16_t)(timer_clk / 1000000UL) - 1;
    uint16_t period = (uint16_t)(1000000UL / RATE_LOOP_HZ) - 1;

    TIM_TimeBaseStructInit(&TB);
    TB.TIM_Prescaler = presc;
    TB.TIM_CounterMode = TIM_CounterMode_Up;
    TB.TIM_Period = period;
    TB.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TB);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);

    /* 角度环 */
    PID_Init(&pidHeight, 0.001f, 0.000001f, 0.0f, -50.0f, 50.0f, 0.02f, -50.0f, 50.0f, 1.0f);
    PID_Init(&pidRoll, 0.0035f, 0.000001f, 0.0003f, -50.0f, 50.0f, 0.02f, -50.0f, 50.0f, 1.0f);
    PID_Init(&pidPitch, 0.0035f, 0.000001f, 0.0003f, -50.0f, 50.0f, 0.02f, -50.0f, 50.0f, 1.0f);
    PID_Init(&pidYaw, 0.001f, 0.000001f, 0.0f, -30.0f, 30.0f, 0.02f, -30.0f, 30.0f, 1.0f);

    /* 速率环 */
    PID_Init(&pidRateRoll, 6.8f, 0.0f, 0.0f, -30.0f, 30.0f, 0.01f, -25.0f, 25.0f, 1.0f);
    PID_Init(&pidRatePitch, 6.8f, 0.0f, 0.0f, -30.0f, 30.0f, 0.01f, -25.0f, 25.0f, 1.0f);
    PID_Init(&pidRateYaw, 2.5f, 0.01f, 0.0f, -20.0f, 20.0f, 0.01f, -20.0f, 20.0f, 1.0f);
}

/* ══════════════════════════════════════════════════════════════
 *  模式切换
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_SetMode(ControlMode_t mode) {
    if (mode > CONTROL_MODE_POSITION)
        return -1;

    curMode = mode;
    moveForward = 0.0f;
    moveRight = 0.0f;
    ResetAllPIDs();

    return mode;
}

ControlMode_t Control_GetMode(void) {
    return curMode;
}

FlightPhase_t Control_GetFlightPhase(void) {
    return curPhase;
}

float Control_GetBaseHeight(void) {
    return baseHeight;
}

void Control_SetSensorFlip(uint8_t flip) {
    rollPitchSign = flip ? -1 : 1;
}

/* ══════════════════════════════════════════════════════════════
 *  解锁 / 锁定
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Arm(void) {
    if (curMode != CONTROL_MODE_DIRECT)
        return -1;
    if (thrustOutput > 1.0f)
        return -1;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    Attitude_GetAltitude(&baseHeight);

    curPhase = FLIGHT_PHASE_GROUNDED;
    isArmed = 1;

    return curPhase;
}

uint8_t Control_IsArmed(void) {
    return isArmed;
}

int8_t Control_Disarm(void) {
    isArmed = 0;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    curPhase = FLIGHT_PHASE_GROUNDED;

    return 0;
}

void Control_EmergencyStop(void) {
    isArmed = 0;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    curPhase = FLIGHT_PHASE_GROUNDED;
}

/* ══════════════════════════════════════════════════════════════
 *  飞行操作
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Takeoff(float relative_height) {
    if (!isArmed)
        return -1;
    if (curPhase != FLIGHT_PHASE_GROUNDED)
        return -1;
    if (relative_height < 0.1f)
        return -1;

    targetHeight = baseHeight + relative_height;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    moveForward = 0.0f;
    moveRight = 0.0f;

    if (curMode < CONTROL_MODE_ALTITUDE) {
        Control_SetMode(CONTROL_MODE_ALTITUDE);
    }

    curPhase = FLIGHT_PHASE_TAKING_OFF;

    return curPhase;
}

int8_t Control_Land(void) {
    if (!isArmed)
        return -1;
    if (curPhase == FLIGHT_PHASE_GROUNDED)
        return -1;
    if (curPhase == FLIGHT_PHASE_LANDING)
        return 0;

    targetHeight = baseHeight;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    moveForward = 0.0f;
    moveRight = 0.0f;

    curPhase = FLIGHT_PHASE_LANDING;

    return curPhase;
}

void Control_Hover(void) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    moveForward = 0.0f;
    moveRight = 0.0f;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
}

/* ══════════════════════════════════════════════════════════════
 *  指令输入
 * ══════════════════════════════════════════════════════════════ */

void Control_SetThrottle(float throttle) {
    if (!isArmed) {
        thrustOutput = 0.0f;
        return;
    }

    if (curMode > CONTROL_MODE_STABILIZED)
        return;

    if (throttle < THROTTLE_DEADBAND)
        throttle = 0.0f;
    thrustOutput = fmaxf(0.0f, fminf(throttle, 100.0f));
}

void Control_SetAttitude(float roll, float pitch, float yaw) {
    if (curMode == CONTROL_MODE_DIRECT)
        return;

    targetRoll = fmaxf(-45.0f, fminf(roll, 45.0f));
    targetPitch = fmaxf(-45.0f, fminf(pitch, 45.0f));
    targetYaw = NormalizeAngle(yaw);
}

void Control_Move(float forward, float right) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    moveForward = fmaxf(-1.0f, fminf(forward, 1.0f));
    moveRight = fmaxf(-1.0f, fminf(right, 1.0f));
}

void Control_SetHeight(float height) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    targetHeight = fmaxf(height, baseHeight);
}
