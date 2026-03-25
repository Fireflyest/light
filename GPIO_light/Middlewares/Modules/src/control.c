#include "control.h"
#include "pwm.h"
#include "attitude.h"
#include <math.h>

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = 50.0f;
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;

void RateControl_Init(uint32_t freq) {
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

    PID_Init(&pidRoll, 4.0f, 0.0f, 0.2f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidPitch, 4.0f, 0.0f, 0.2f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidYaw, 2.0f, 0.0f, 0.1f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidHeight, 1.0f, 0.0f, 0.1f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidRateRoll, 0.15f, 0.001f, 0.002f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.15f, 0.001f, 0.002f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRateYaw, 0.10f, 0.0005f, 0.001f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);
}

// 内环执行（在 TIM4 中断上下文，尽量短小）
// 读 gyro -> LPF -> rate PID -> motor mixing -> 写 pwmDutyBuffer
void RateControl_Loop(void) {
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
    float m0 = throttle + pitchCtrl + rollCtrl + yawCtrl;
    float m1 = throttle + pitchCtrl - rollCtrl - yawCtrl;
    float m2 = throttle - pitchCtrl + rollCtrl - yawCtrl;
    float m3 = throttle - pitchCtrl - rollCtrl + yawCtrl;
    float m[4] = { m0, m1, m2, m3 };

    float maxv = fmaxf(fmaxf(m[0], m[1]), fmaxf(m[2], m[3]));
    float minv = fminf(fminf(m[0], m[1]), fminf(m[2], m[3]));

    // 如果最小值低于下限，先整体上移
    const float OUT_MIN = 0.0f, OUT_MAX = 100.0f;
    if (minv < OUT_MIN) {
        float shift = OUT_MIN - minv;
        for (int i = 0; i < 4; i++) m[i] += shift;
        maxv += shift;
        minv = OUT_MIN;
    }

    // 如果最大值超上限，按比例缩放（保持相对差值）
    if (maxv > OUT_MAX && maxv > 0.0f) {
        float scale = OUT_MAX / maxv;
        for (int i = 0; i < 4; i++) m[i] *= scale;
    }

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

void RateControl_TargetAttitude(sm_quat_t q_target, float h_target) {
    sm_quat_t q_corr;
    Attitude_GetQuat(q_corr);
    // 计算目标四元数的共轭
    Spatial_QuatConjugate(q_target);
    // 计算误差四元数：q_error = q_corr * q_target^*
    sm_quat_t q_error;
    Spatial_QuatMultiply(q_error, q_corr, q_target);
    // 从误差四元数提取旋转轴和角度
    float angle = 2.0f * acosf(q_error[0]);
    sm_vec3_t axis = {q_error[1], q_error[2], q_error[3]};
    Spatial_Vec3Normalize(axis);
    Spatial_Vec3MultiplyScalar(axis, angle);
    float current_height;
    Attitude_GetAltitude(&current_height);
    // PID 控制
    rateSetRoll  = PID_Update(&pidRoll, axis[0], 0.0f, 1.0f / (float)RATE_LOOP_HZ);
    rateSetPitch = PID_Update(&pidPitch, axis[1], 0.0f, 1.0f / (float)RATE_LOOP_HZ);
    rateSetYaw   = PID_Update(&pidYaw, axis[2], 0.0f, 1.0f / (float)RATE_LOOP_HZ);
    float heightOutput = PID_Update(&pidHeight, h_target, current_height, 1.0f / (float)RATE_LOOP_HZ);
    thrustOutput = baseThrottle + heightOutput;
}