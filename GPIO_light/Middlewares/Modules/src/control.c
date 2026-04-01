#include "control.h"
#include "pwm.h"
#include "attitude.h"
#include <math.h>

/* ══════════════════════════════════════════════════════════════
 *  PID & 公开变量
 * ══════════════════════════════════════════════════════════════ */

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = 50.0f;
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;

/* ══════════════════════════════════════════════════════════════
 *  内部状态
 * ══════════════════════════════════════════════════════════════ */

static ControlMode_t  curMode   = CONTROL_MODE_DIRECT;
static FlightPhase_t  curPhase  = FLIGHT_PHASE_GROUNDED;
static uint8_t        isArmed   = 0;
static int8_t rollPitchSign = 1; /* 1=Z-down, -1=Z-up */

static float baseHeight    = 0.0f;   /* 上电基准高度 (m) */
static float targetRoll    = 0.0f;   /* 目标横滚角  (°)  */
static float targetPitch   = 0.0f;   /* 目标俯仰角  (°)  */
static float targetYaw     = 0.0f;   /* 目标偏航角  (°)  */
static float targetHeight  = 0.0f;   /* 目标高度    (m)  */
static float moveForward   = 0.0f;   /* 前进指令 [-1,1]  */
static float moveRight     = 0.0f;   /* 右移指令 [-1,1]  */

/* ──────────────────────────────────────────────────────────────
 *  内部工具
 * ────────────────────────────────────────────────────────────── */

static float NormalizeAngle(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static void ClampMotors(float m[4])
{
    const float OUT_MIN = 0.0f;
    const float OUT_MAX = 100.0f;

    float maxv = fmaxf(fmaxf(m[0], m[1]), fmaxf(m[2], m[3]));
    float minv = fminf(fminf(m[0], m[1]), fminf(m[2], m[3]));

    if (minv < OUT_MIN) {
        float shift = OUT_MIN - minv;
        for (int i = 0; i < 4; i++) m[i] += shift;
        maxv += shift;
    }
    if (maxv > OUT_MAX && maxv > 0.0f) {
        float scale = OUT_MAX / maxv;
        for (int i = 0; i < 4; i++) m[i] *= scale;
    }
}

static void ResetAllPIDs(void)
{
    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

static void ResetAllTargets(void)
{
    targetRoll   = 0.0f;
    targetPitch  = 0.0f;
    targetYaw    = 0.0f;
    targetHeight = baseHeight;
    moveForward  = 0.0f;
    moveRight    = 0.0f;
}

/* ──────────────────────────────────────────────────────────────
 *  外环（姿态 + 高度）— 由主循环以 200 Hz 调用
 * ────────────────────────────────────────────────────────────── */

void ControlAttitude_Loop(void)
{
    if (!isArmed) {
        thrustOutput = 0.0f;
        rateSetRoll  = 0.0f;
        rateSetPitch = 0.0f;
        rateSetYaw   = 0.0f;
        return;
    }

    const float dt = 1.0f / (float)ATTITUDE_LOOP_HZ;

    /* ── 读取当前姿态 ─────────────────────────────── */
    float curRoll, curPitch, curYaw;
    Attitude_GetEuler(&curRoll, &curPitch, &curYaw);

    float curHeight;
    Attitude_GetAltitude(&curHeight);

    /* ── 降落状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_LANDING) {
        /* 目标高度已由 Control_Land() 渐降，这里只做接地检测 */
        if (curHeight < baseHeight + 0.05f) {
            /* 接地：自动锁定 */
            Control_Disarm();
            return;  /* Disarm 后 isArmed=0，下帧直接走零输出 */
        }
    }

    /* ── 起飞状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_TAKING_OFF) {
        /* 高度达到目标 ±10cm 视为起飞完成 */
        if (fabsf(curHeight - targetHeight) < 0.10f) {
            curPhase = FLIGHT_PHASE_IN_FLIGHT;
        }
    }

    /* ── 高度环 ──────────────────────────────────── */
    if (curMode >= CONTROL_MODE_ALTITUDE) {
        thrustOutput = baseThrottle
                     + PID_Update(&pidHeight, targetHeight, curHeight, dt);
        thrustOutput = fmaxf(0.0f, fminf(thrustOutput, 100.0f));
    }
    /* DIRECT / STABILIZED: thrustOutput 由 Control_SetThrottle() 直接写入 */

    /* ── 姿态环 ──────────────────────────────────── */

    /* DIRECT 模式：角度环输出被忽略，内环直接接收摇杆速率 */
    if (curMode == CONTROL_MODE_DIRECT) {
        /* 角度环仍在跑（保持积分连续），但不赋给 rateSet */
        return;
    }

    float effRoll  = targetRoll;
    float effPitch = targetPitch;

    /* ALTITUDE 及以上：平移指令叠加到目标姿态 */
    if (curMode >= CONTROL_MODE_ALTITUDE) {
        effPitch -= moveForward * 25.0f;      /* 前进 = 负俯仰, 最大 ±25° */
        effRoll  += moveRight   * 25.0f;      /* 右移 = 正横滚            */
    }

    /* 角度 PID → 角速率设定点
     *
     * Z-down: rollPitchSign = +1
     *   右倾 → curRoll > 0 → PID 输出负 → rateSetRoll 负 → 速率环修正 ✓
     *
     * Z-up:   rollPitchSign = -1
     *   右倾 → curRoll < 0 → PID 输出正 → 取反后负 → 速率环修正 ✓
     */
    rateSetRoll = (float)rollPitchSign * PID_Update(&pidRoll, effRoll, curRoll, dt);
    rateSetPitch = (float)rollPitchSign * PID_Update(&pidPitch, effPitch, curPitch, dt);

    /* 偏航：短角误差归一化 */
    float yawErr = NormalizeAngle(targetYaw - curYaw);
    rateSetYaw   = PID_Update(&pidYaw, 0.0f, -yawErr, dt);
}

/* ──────────────────────────────────────────────────────────────
 *  内环（速率）— 在 TIM4 中断中以 1000 Hz 调用
 * ────────────────────────────────────────────────────────────── */

void ControlMotor_Loop(void)
{
    const float dt = 1.0f / (float)RATE_LOOP_HZ;

    sm_vec3_t gyro;
    Attitude_GetGyro(gyro);
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    if (rollPitchSign < 0) {
        gy = -gy;
    }

    float rollCtrl  = PID_Update(&pidRateRoll,  rateSetRoll,  gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, rateSetPitch, gy, dt);
    float yawCtrl   = PID_Update(&pidRateYaw,   rateSetYaw,   gz, dt);

    float throttle = thrustOutput;

    /* ═══════════════════════════════════════════════════════
     *  电机混控
     *
     *  布局:        X(前)
     *               ^
     *          M3   |   M1
     *     ----------+--------> Y(右)
     *          M4   |   M2
     * ═══════════════════════════════════════════════════════ */
    float m[4];
    m[0] = throttle + pitchCtrl + rollCtrl + yawCtrl;   /* M1 前右 CW  */
    m[1] = throttle - pitchCtrl + rollCtrl - yawCtrl;   /* M2 后右 CCW */
    m[2] = throttle + pitchCtrl - rollCtrl - yawCtrl;   /* M3 前左 CCW */
    m[3] = throttle - pitchCtrl - rollCtrl + yawCtrl;   /* M4 后左 CW  */

    ClampMotors(m);

    pwmDutyBuffer[0] = PWM_Map_Percent(m[0]);
    pwmDutyBuffer[1] = PWM_Map_Percent(m[1]);
    pwmDutyBuffer[2] = PWM_Map_Percent(m[2]);
    pwmDutyBuffer[3] = PWM_Map_Percent(m[3]);
}

/* ──────────────────────────────────────────────────────────────
 *  初始化
 * ────────────────────────────────────────────────────────────── */

void Control_Init(float height_init)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TB;
    uint32_t timer_clk = SystemCoreClock;
    uint16_t presc  = (uint16_t)(timer_clk / 1000000UL) - 1;
    uint16_t period = (uint16_t)(1000000UL / RATE_LOOP_HZ) - 1;

    TIM_TimeBaseStructInit(&TB);
    TB.TIM_Prescaler       = presc;
    TB.TIM_CounterMode     = TIM_CounterMode_Up;
    TB.TIM_Period          = period;
    TB.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TB);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);

    /* 角度环 PID */
    PID_Init(&pidRoll,    6.0f, 0.0f,    1.0f,   -300.0f, 300.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidPitch,   6.0f, 0.0f,    1.0f,   -300.0f, 300.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidYaw,     5.0f, 0.0f,    0.5f,   -200.0f, 200.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidHeight,  1.0f, 0.0f,    0.1f,   -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);

    /* 速率环 PID */
    PID_Init(&pidRateRoll,  0.5f, 0.01f,  0.005f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.5f, 0.01f,  0.005f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRateYaw,   0.3f, 0.005f, 0.003f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);

    baseHeight = height_init;
}

/* ══════════════════════════════════════════════════════════════
 *  模式查询与切换
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_SetMode(ControlMode_t mode)
{
    if (mode > CONTROL_MODE_POSITION) return -1;

    /* 传感器检查 */
    if (mode >= CONTROL_MODE_ALTITUDE) {
        /* TODO: 检查高度传感器是否正常 */
    }
    if (mode >= CONTROL_MODE_VELOCITY) {
        /* TODO: 检查速度估计是否可用 */
    }

    curMode = mode;

    /* 切模式时清前馈 + 复位 PID */
    moveForward = 0.0f;
    moveRight   = 0.0f;
    ResetAllPIDs();

    return 0;
}

ControlMode_t Control_GetMode(void)
{
    return curMode;
}

FlightPhase_t Control_GetFlightPhase(void)
{
    return curPhase;
}

float Control_GetBaseHeight(void)
{
    return baseHeight;
}

/* ══════════════════════════════════════════════════════════════
 *  传感器朝向
 * ══════════════════════════════════════════════════════════════ */

void Control_SetSensorFlip(uint8_t flip) {
    rollPitchSign = flip ? -1 : 1;
}

/* ══════════════════════════════════════════════════════════════
 *  解锁 / 锁定
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Arm(void)
{
    if (curMode != CONTROL_MODE_DIRECT) return -1;
    if (thrustOutput > 1.0f)            return -1;

    isArmed = 1;

    /* 记录上电基准高度 */
    Attitude_GetAltitude(&baseHeight);

    /* 复位全部状态 */
    ResetAllPIDs();
    ResetAllTargets();
    thrustOutput = 0.0f;
    rateSetRoll  = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw   = 0.0f;

    curPhase = FLIGHT_PHASE_GROUNDED;

    return 0;
}

int8_t Control_Disarm(void)
{
    if (curPhase != FLIGHT_PHASE_GROUNDED) return -1;

    isArmed = 0;
    thrustOutput = 0.0f;
    rateSetRoll  = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw   = 0.0f;

    ResetAllPIDs();
    ResetAllTargets();

    return 0;
}

void Control_EmergencyStop(void)
{
    isArmed      = 0;
    thrustOutput = 0.0f;
    rateSetRoll  = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw   = 0.0f;

    curPhase = FLIGHT_PHASE_GROUNDED;

    /* 立刻写零到电机（不等下一周期） */
    pwmDutyBuffer[0] = 0;
    pwmDutyBuffer[1] = 0;
    pwmDutyBuffer[2] = 0;
    pwmDutyBuffer[3] = 0;
}

/* ══════════════════════════════════════════════════════════════
 *  飞行操作
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Takeoff(float relative_height)
{
    if (!isArmed)                      return -1;
    if (curPhase != FLIGHT_PHASE_GROUNDED) return -1;
    if (relative_height < 0.1f)        return -1;

    /* 目标高度 = 上电基准 + 相对高度 */
    targetHeight = baseHeight + relative_height;
    targetRoll   = 0.0f;
    targetPitch  = 0.0f;
    moveForward  = 0.0f;
    moveRight    = 0.0f;

    /* 自动提升到 ALTITUDE（如果当前更低） */
    if (curMode < CONTROL_MODE_ALTITUDE) {
        Control_SetMode(CONTROL_MODE_ALTITUDE);
    }

    curPhase = FLIGHT_PHASE_TAKING_OFF;

    return 0;
}

int8_t Control_Land(void)
{
    if (!isArmed)                            return -1;
    if (curPhase == FLIGHT_PHASE_GROUNDED)   return -1;
    if (curPhase == FLIGHT_PHASE_LANDING)    return 0;   /* 已在降落 */

    /* 目标高度降至基准高度（即地面） */
    targetHeight = baseHeight;
    targetRoll   = 0.0f;
    targetPitch  = 0.0f;
    moveForward  = 0.0f;
    moveRight    = 0.0f;

    curPhase = FLIGHT_PHASE_LANDING;
    /* 接地检测在 ControlAttitude_Loop() 中自动完成 */

    return 0;
}

void Control_Hover(void)
{
    if (curMode < CONTROL_MODE_ALTITUDE) return;

    moveForward = 0.0f;
    moveRight   = 0.0f;
    targetRoll  = 0.0f;
    targetPitch = 0.0f;
    /* targetHeight 保持不变 */
}

/* ══════════════════════════════════════════════════════════════
 *  指令输入
 * ══════════════════════════════════════════════════════════════ */

void Control_SetThrottle(float throttle)
{
    if (!isArmed) {
        thrustOutput = 0.0f;
        return;
    }

    /* 仅 DIRECT / STABILIZED 模式生效 */
    if (curMode > CONTROL_MODE_STABILIZED) return;

    if (throttle < 2.0f) throttle = 0.0f;    /* 2% 死区 */
    thrustOutput = fmaxf(0.0f, fminf(throttle, 100.0f));
}

void Control_SetAttitude(float roll, float pitch, float yaw)
{
    if (curMode == CONTROL_MODE_DIRECT) return;

    targetRoll  = fmaxf(-45.0f, fminf(roll,  45.0f));
    targetPitch = fmaxf(-45.0f, fminf(pitch, 45.0f));
    targetYaw   = NormalizeAngle(yaw);
}

void Control_Move(float forward, float right)
{
    if (curMode < CONTROL_MODE_ALTITUDE) return;

    moveForward = fmaxf(-1.0f, fminf(forward, 1.0f));
    moveRight   = fmaxf(-1.0f, fminf(right,   1.0f));
}

void Control_SetHeight(float height)
{
    if (curMode < CONTROL_MODE_ALTITUDE) return;

    targetHeight = fmaxf(height, baseHeight);
}
