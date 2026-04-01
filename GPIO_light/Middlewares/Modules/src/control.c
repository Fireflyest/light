#include "control.h"
#include "pwm.h"
#include "attitude.h"
#include <math.h>

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = 50.0f;
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;

static ControlMode_t  currentMode       = CONTROL_MODE_MANUAL;
static FlightMode_t   currentFlightMode = FLIGHT_MODE_LAND;
static uint8_t        isArmed           = 0;

static float targetRoll    = 0.0f;   /* 目标横滚角  (°)  */
static float targetPitch   = 0.0f;   /* 目标俯仰角  (°)  */
static float targetYaw     = 0.0f;   /* 目标偏航角  (°)  */
static float targetHeight  = 0.0f;   /* 目标高度    (m)  */
static float moveForward   = 0.0f;   /* 前进指令 [-1,1]  */
static float moveRight     = 0.0f;   /* 右移指令 [-1,1]  */

/* ──────────────────────────────────────────────────────────────
 * 内部辅助
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  将角度归一化到 (-180, 180]
 */
static float NormalizeAngle(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/**
 * @brief  安全裁剪：电机输出低于怠速时整体抬升，超限则等比缩放
 * @note   复用 ControlMotor_Loop 中已有的逻辑
 */
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

/* ──────────────────────────────────────────────────────────────
 * 外环（姿态 + 高度）— 建议由主循环以 200 Hz 调用
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  外环控制：读姿态 → 姿态 PID → 速率设定点 → 调用内环
 *
 *  调用关系：
 *    main loop (200 Hz)
 *      └→ ControlAttitude_Loop()
 *            ├─ 读当前姿态 (roll/pitch/yaw)
 *            ├─ 姿态 PID → rateSetRoll / rateSetPitch / rateSetYaw
 *            ├─ 高度 PID → thrustOutput
 *            └─ ControlMotor_Loop()  ← 内环（实际写电机）
 */
void ControlAttitude_Loop(void)
{
    if (!isArmed) {
        /* 未解锁：强制零输出 */
        thrustOutput = 0.0f;
        rateSetRoll  = 0.0f;
        rateSetPitch = 0.0f;
        rateSetYaw   = 0.0f;
        return;
    }

    const float dt = 1.0f / 200.0f;          /* 外环周期 5 ms */

    /* ── 读取当前姿态 ─────────────────────────────── */
    float curRoll, curPitch, curYaw;
    Attitude_GetEuler(&curRoll, &curPitch, &curYaw);                    /* att[0]=roll, att[1]=pitch, att[2]=yaw */

    float curHeight;
    Attitude_GetAltitude(&curHeight);

    /* ── 高度环 ──────────────────────────────────── */
    if (currentFlightMode == FLIGHT_MODE_TAKEOFF ||
        currentFlightMode == FLIGHT_MODE_HOVER  ||
        currentFlightMode == FLIGHT_MODE_LAND) {
        thrustOutput = baseThrottle
                     + PID_Update(&pidHeight, targetHeight, curHeight, dt);
        thrustOutput = fmaxf(0.0f, fminf(thrustOutput, 100.0f));
    }
    /* MANUAL 模式下 thrustOutput 由 Control_SetThrottle() 直接写入 */

    /* ── 姿态环 ──────────────────────────────────── */
    float effRoll  = targetRoll;
    float effPitch = targetPitch;

    /* 前进/右移 → 叠加到目标俯仰/横滚 */
    if (currentMode == CONTROL_MODE_VELOCITY ||
        currentMode == CONTROL_MODE_POSITION) {
        effPitch -= moveForward * 25.0f;      /* 前进 = 负俯仰, 最大 ±25° */
        effRoll  += moveRight   * 25.0f;      /* 右移 = 正横滚            */
    }

    /* 角度 PID → 角速率设定点 */
    rateSetRoll  = PID_Update(&pidRoll,  effRoll,  curRoll,  dt);
    rateSetPitch = PID_Update(&pidPitch, effPitch, curPitch, dt);

    /* 偏航：短角误差直接归一化 */
    float yawErr = NormalizeAngle(targetYaw - curYaw);
    rateSetYaw   = PID_Update(&pidYaw, 0.0f, -yawErr, dt);
}


// 内环执行（在 TIM4 中断上下文，尽量短小）
// 读 gyro -> LPF -> rate PID -> motor mixing -> 写 pwmDutyBuffer
void ControlMotor_Loop(void) {
    const float dt = 1.0f / (float)RATE_LOOP_HZ;
    // 读取陀螺（优先使用 imu_ekf 的速率字段，如果没有，回退到原始 mpuDataBuffer）
    sm_vec3_t gyro;
    Attitude_GetGyro(gyro);
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    float rollCtrl  = PID_Update(&pidRateRoll,  rateSetRoll,  gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, rateSetPitch, gy, dt);
    float yawCtrl   = PID_Update(&pidRateYaw,   rateSetYaw,   gz, dt);

    // motor mixing (百分比单位假设 0..100)，thrustOutput 由主循环设置
    float throttle = thrustOutput;
    /* ═══════════════════════════════════════════════════════
     *  电机混控
     *
     *  布局:        X(前)
     *               ^
     *          M3   |   M1
     *     ----------+--------> Y(右)
     *          M4   |   M2
     *
     * 
     * ═══════════════════════════════════════════════════════ */
    float m[4];
    m[0] = throttle + pitchCtrl + rollCtrl + yawCtrl;   /* M1 前右 CW  */
    m[1] = throttle - pitchCtrl + rollCtrl - yawCtrl;   /* M2 后右 CCW */
    m[2] = throttle + pitchCtrl - rollCtrl - yawCtrl;   /* M3 前左 CCW */
    m[3] = throttle - pitchCtrl - rollCtrl + yawCtrl;   /* M4 后左 CW  */

    ClampMotors(m);

    // 写入 PWM 缓冲（短临界区）
    pwmDutyBuffer[0] = PWM_Map_Percent(m[0]);
    pwmDutyBuffer[1] = PWM_Map_Percent(m[1]);
    pwmDutyBuffer[2] = PWM_Map_Percent(m[2]);
    pwmDutyBuffer[3] = PWM_Map_Percent(m[3]);
    // TIM_SetCompare1(TIM3, PWM_Map_Percent(m[0]));
    // TIM_SetCompare2(TIM3, PWM_Map_Percent(m[1]));
    // TIM_SetCompare3(TIM3, PWM_Map_Percent(m[2]));
    // TIM_SetCompare4(TIM3, PWM_Map_Percent(m[3]));
}

void Control_Init(uint32_t freq) {
    // 启动 TIM4 时钟做 1kHz 更新
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TB;
    uint32_t timer_clk = SystemCoreClock; // F401 为 84MHz
    uint16_t presc = (uint16_t)(timer_clk / 1000000UL) - 1; // timer at 1MHz
    uint16_t period = (uint16_t)(1000000UL / freq) - 1;

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

    /* 角度环 PID */
    PID_Init(&pidRoll,   6.0f,  0.0f,   1.0f,   -300.0f, 300.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidPitch,  6.0f,  0.0f,   1.0f,   -300.0f, 300.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidYaw,     5.0f,  0.0f,   0.5f,   -200.0f, 200.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidHeight,  1.0f,  0.0f,   0.1f,   -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);

    /* 速率环 PID */
    PID_Init(&pidRateRoll,  0.5f,   0.01f,  0.005f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.5f,   0.01f,  0.005f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRateYaw,   0.3f,   0.005f, 0.003f, -80.0f, 80.0f, 0.01f, -400.0f, 400.0f, 1.0f);
}

/* ──────────────────────────────────────────────────────────────
 * 公开接口实现
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  切换控制模式
 * @param  new_mode  CONTROL_MODE_MANUAL / _ATTITUDE / _VELOCITY / _POSITION
 */
void Control_SetMode(uint8_t new_mode)
{
    switch (new_mode) {
    case CONTROL_MODE_MANUAL:
    case CONTROL_MODE_ATTITUDE:
    case CONTROL_MODE_VELOCITY:
    case CONTROL_MODE_POSITION:
        currentMode = (ControlMode_t)new_mode;
        break;
    default:
        /* 未知模式 → 强制回退手动 */
        currentMode = CONTROL_MODE_MANUAL;
        break;
    }

    /* 切模式时复位前馈，防止残留 */
    moveForward = 0.0f;
    moveRight   = 0.0f;

    /* 清积分项，避免模式切换瞬间跳变 */
    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

/**
 * @brief  手动模式下直接设置油门百分比
 * @param  throttle  0 ~ 100
 *
 * @note   仅在 CONTROL_MODE_MANUAL 时生效；
 *         其他模式下油门由高度环自动计算。
 */
void Control_SetThrottle(float throttle)
{
    if (!isArmed) {
        thrustOutput = 0.0f;
        return;
    }

    /* 保留 2% 死区：防止怠速旋转 */
    if (throttle < 2.0f) throttle = 0.0f;
    thrustOutput = fmaxf(0.0f, fminf(throttle, 100.0f));
}

/**
 * @brief  设置目标悬停高度
 * @param  height  目标高度 (m)，≥ 0
 */
void Control_SetHeight(float height)
{
    targetHeight = fmaxf(height, 0.0f);
}

/**
 * @brief  设置平面移动指令
 * @param  forward  前进速度归一化 [-1, 1]（正值 = 前进）
 * @param  right    右移速度归一化 [-1, 1]（正值 = 右移）
 *
 * @note   实际效果：在 ControlAttitude_Loop 中叠加到目标俯仰/横滚
 */
void Control_Move(float forward, float right)
{
    moveForward = fmaxf(-1.0f, fminf(forward, 1.0f));
    moveRight   = fmaxf(-1.0f, fminf(right,   1.0f));
}

/**
 * @brief  设置目标姿态角
 * @param  roll   目标横滚 (°)，建议 ±45
 * @param  pitch  目标俯仰 (°)，建议 ±45
 * @param  yaw    目标偏航 (°)，会自动归一化到 (-180,180]
 */
void Control_SetAttitude(float roll, float pitch, float yaw)
{
    targetRoll  = fmaxf(-45.0f, fminf(roll,  45.0f));
    targetPitch = fmaxf(-45.0f, fminf(pitch, 45.0f));
    targetYaw   = NormalizeAngle(yaw);
}

/**
 * @brief  解锁电机
 *
 *  安全条件：
 *    1. 必须处于 MANUAL 模式（防止误触自动起飞）
 *    2. 油门必须为 0
 *    3. 复位所有 PID 和目标值
 */
void Control_Arm(void)
{
    if (currentMode != CONTROL_MODE_MANUAL) {
        return;  /* 非手动模式不允许解锁 */
    }
    if (thrustOutput > 1.0f) {
        return;  /* 油门不在零位 */
    }

    isArmed = 1;

    /* 复位所有环路状态 */
    targetRoll    = 0.0f;
    targetPitch   = 0.0f;
    targetYaw     = 0.0f;
    targetHeight  = 0.0f;
    moveForward   = 0.0f;
    moveRight     = 0.0f;
    thrustOutput  = 0.0f;
    rateSetRoll   = 0.0f;
    rateSetPitch  = 0.0f;
    rateSetYaw    = 0.0f;

    currentFlightMode = FLIGHT_MODE_LAND;

    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

/**
 * @brief  紧急停止 — 立即锁电机、切断油门、冻结所有输出
 *
 *  最高优先级，不做任何条件检查。
 *  建议绑定到独立硬件中断（如遥控器失控检测）。
 */
void Control_EmergencyStop(void)
{
    isArmed  = 0;
    thrustOutput = 0.0f;
    rateSetRoll  = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw   = 0.0f;

    currentFlightMode = FLIGHT_MODE_LAND;

    /* 立刻写零到电机（不等下一周期） */
    pwmDutyBuffer[0] = 0;
    pwmDutyBuffer[1] = 0;
    pwmDutyBuffer[2] = 0;
    pwmDutyBuffer[3] = 0;
}

/**
 * @brief  切换飞行阶段
 * @param  mode  FLIGHT_MODE_TAKEOFF / _HOVER / _LAND / _MANUAL
 *
 *  行为摘要：
 *    TAKEOFF  → 目标高度设为 1.0 m，高度环接管油门
 *    HOVER    → 保持当前目标高度，姿态环归零
 *    LAND     → 目标高度缓降至 0，到位后自动切回 MANUAL
 *    MANUAL   → 高度环退出，油门交还给用户
 */
void Control_FlightMode(uint8_t mode)
{
    if (!isArmed && mode != FLIGHT_MODE_LAND) {
        return;  /* 未解锁只能降落 / 待机 */
    }

    FlightMode_t prev = currentFlightMode;
    currentFlightMode = (FlightMode_t)mode;

    switch (currentFlightMode) {

    case FLIGHT_MODE_TAKEOFF:
        /* 若尚未指定高度，默认升到 1 m */
        if (targetHeight < 0.3f) {
            targetHeight = 1.0f;
        }
        targetRoll  = 0.0f;
        targetPitch = 0.0f;
        break;

    case FLIGHT_MODE_HOVER:
        /* 锁定当前位置和高度 */
        targetRoll  = 0.0f;
        targetPitch = 0.0f;
        moveForward = 0.0f;
        moveRight   = 0.0f;
        /* targetHeight 保持不变 */
        break;

    case FLIGHT_MODE_LAND:
        /* 缓降目标高度；外环检测到高度 < 0.05 m 后可自动 disarm */
        targetHeight = 0.0f;
        targetRoll   = 0.0f;
        targetPitch  = 0.0f;
        moveForward  = 0.0f;
        moveRight    = 0.0f;
        break;

    case FLIGHT_MODE_MANUAL:
        /* 高度环退出，油门交还用户 */
        moveForward = 0.0f;
        moveRight   = 0.0f;
        break;

    default:
        currentFlightMode = prev;   /* 未知模式，回退 */
        break;
    }
}
