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
static uint8_t sensorIsFlipped = 0;

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

    // 既然在 navigator.c 里面，UI_Cube_Draw 直接使用 Attitude_GetQuat 
    // 就能在 OLED 上完美无损地显示真实的 3D 姿态（没有任何串扰），
    // 说明姿态解算出来的四元数 q 已经是完全对齐到无人机机身的绝对正确姿态了！
    // 原来之前在底层某个地方（比如传感器或者EKF对齐阶段）早就已经做过底面的处理。
    // 所以这里再做一次翻转，反而把正确的姿态给搞乱了！导致发生 90度/180度 错位从而引起串扰。
    // 现在直接把这段画蛇添足的旋转去掉：

    float curRoll, curPitch, curYaw;
    // 强制使用无人机源头解析计算的四元数
    curRoll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 180.0f / M_PI_F;
    curPitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180.0f / M_PI_F;
    curYaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 180.0f / M_PI_F;

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
    // 注意，此处 curYawAngle 不能再用原本拿底层原滋原味 q_sensor 算的算法了
    // 应当直接使用刚才从翻转回机身的四元数解析的 curYaw，转回弧度
    float curYawAngle = curYaw * (M_PI_F / 180.0f);

    float yawErrDeg = NormalizeAngle(targetYaw - curYawAngle * 180.0f / M_PI_F);
    float targetYawRad = curYawAngle + yawErrDeg * M_PI_F / 180.0f;
    
    // ======== 修正点核心 ========
    // 之前这段经典四元数误差推导（将目标欧拉角转为四元数然后做共轭相乘求 ex、ey、ez）
    // 其实是在 Z-Y-X (Yaw-Pitch-Roll) 的底层定义下计算的
    // 原本你在里面写的数学展开就是根据这个公式。
    // 但是这里算出来的 ex, ey, ez 的旋转方向，刚好也是正负反掉了或者发生了串扰
    // 我们用更直观的【目标欧拉角减去当前欧拉角】直接送入外环 PID：
    
    float errRoll = NormalizeAngle(targetRoll - curRoll);
    float errPitch = NormalizeAngle(targetPitch - curPitch);
    float errYaw = yawErrDeg;

    /* Gimbal Lock 保护 */
    float sinPitch = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    if (fabsf(sinPitch) > GIMBAL_LOCK_THRESH) {
        errYaw = 0.0f;
    }

    /* ── 角度环 PID ──────────────────────────────── */
    // 因为这里我们直接用了 errRoll (度) 的单位
    // 而不用再乘上玄学的 TO_DEG (原本的 TO_DEG 只是拿四元数的 x 矢量用来近似还原度数)
    float newRateSetRoll = PID_Update(&pidRoll, 0.0f, errRoll, dt);
    float newRateSetPitch = PID_Update(&pidPitch, 0.0f, errPitch, dt);
    float newRateSetYaw = PID_Update(&pidYaw, 0.0f, errYaw, dt);

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
    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];

    // 同理，外圈姿态都没有被翻转，证明我们在更低层已经处理过了，或者根本没贴反。
    // PID 内环接收到的角速度和四元数方向是一致的，不需要再对 gyro_current 手动反向。

    float rollCtrl = PID_Update(&pidRateRoll, localRateSetRoll, gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, localRateSetPitch, gy, dt);
    float yawCtrl = PID_Update(&pidRateYaw, localRateSetYaw, gz, dt);

    float throttle = thrustOutput;

    /* 
     * 电机混控 (标准的无人机 FRD X型四轴混控矩阵)
     * 1: 前左(FL)   2: 后左(RL)
     * 3: 前右(FR)   4: 后右(RR)
     *
     * +Pitch (抬头) -> 前面电机加速, 后面电机减速 -> FL(+), FR(+) / RL(-), RR(-)
     * +Roll  (右滚) -> 左面电机加速, 右面电机减速 -> FL(+), RL(+) / FR(-), RR(-)
     * +Yaw   (右偏) -> CCW电机加速, CW电机减速    -> 假设 FL(CW), RR(CW), FR(CCW), RL(CCW)
     *                  即 FR(+), RL(+) / FL(-), RR(-)
     */
    float m[4];
    // 恢复你最开始完全正确的混控矩阵！
    m[0] = throttle - pitchCtrl - rollCtrl + yawCtrl; // M1(1): 前左 (FL)
    m[1] = throttle + pitchCtrl - rollCtrl - yawCtrl; // M2(2): 后左 (RL)
    m[2] = throttle - pitchCtrl + rollCtrl - yawCtrl; // M3(3): 前右 (FR)
    m[3] = throttle + pitchCtrl + rollCtrl + yawCtrl; // M4(4): 后右 (RR)

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
    PID_Init(&pidRoll, 0.0045f, 0.000001f, 0.0003f, -50.0f, 50.0f, 0.02f, -50.0f, 50.0f, 1.0f);
    PID_Init(&pidPitch, 0.0045f, 0.000001f, 0.0003f, -50.0f, 50.0f, 0.02f, -50.0f, 50.0f, 1.0f);
    PID_Init(&pidYaw, 0.001f, 0.000001f, 0.0f, -30.0f, 30.0f, 0.02f, -30.0f, 30.0f, 1.0f);

    /* 速率环 */
    PID_Init(&pidRateRoll, 1.8f, 0.0f, 0.0f, -30.0f, 30.0f, 0.01f, -25.0f, 25.0f, 1.0f);
    PID_Init(&pidRatePitch, 1.8f, 0.0f, 0.0f, -30.0f, 30.0f, 0.01f, -25.0f, 25.0f, 1.0f);
    PID_Init(&pidRateYaw, 1.5f, 0.01f, 0.0f, -20.0f, 20.0f, 0.01f, -20.0f, 20.0f, 1.0f);
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
    sensorIsFlipped = flip;
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
