#include "control.h"
#include "pwm.h"
#include "attitude.h"
#include <math.h>

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = .0f;
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;
LowPass_Filter_t gyroFilt[3];

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

    // 初始化陀螺滤波器
    float dt = 1.0f / (float)RATE_LOOP_HZ;
    float alpha = dt / (GYRO_TAU + dt);
    for (int i = 0; i < 3; i++) {
        LowPass_Filter_Init((LowPass_Filter_t*)&gyroFilt[i], alpha, 0.0f);
    }
}



// 内环执行（在 TIM4 中断上下文，尽量短小）
// 读 gyro -> LPF -> rate PID -> motor mixing -> 写 pwmDutyBuffer
void RateControl_Loop(void) {
    const float dt = 1.0f / (float)RATE_LOOP_HZ;
    // 读取陀螺（优先使用 imu_ekf 的速率字段，如果没有，回退到原始 mpuDataBuffer）
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    gx = imu_ekf.gyro_corr[0];
    gy = imu_ekf.gyro_corr[1];
    gz = imu_ekf.gyro_corr[2];

    LowPass_UpdateWithTau((LowPass_Filter_t*)&gyroFilt[0], gx, GYRO_TAU, dt);
    LowPass_UpdateWithTau((LowPass_Filter_t*)&gyroFilt[1], gy, GYRO_TAU, dt);
    LowPass_UpdateWithTau((LowPass_Filter_t*)&gyroFilt[2], gz, GYRO_TAU, dt);

    // 内环 PID（setpoint: rateSet*, measurement: gyro_lp[*]）
    float rollCtrl = PID_Update(&pidRateRoll, rateSetRoll, gyroFilt[0].output, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, rateSetPitch, gyroFilt[1].output, dt);
    float yawCtrl = PID_Update(&pidRateYaw, rateSetYaw, gyroFilt[2].output, dt);

    // motor mixing (百分比单位假设 0..100)，thrustOutput 由主循环设置
    float throttle = thrustOutput;
    float m0 = throttle + pitchCtrl + rollCtrl + yawCtrl;
    float m1 = throttle + pitchCtrl - rollCtrl - yawCtrl;
    float m2 = throttle - pitchCtrl + rollCtrl - yawCtrl;
    float m3 = throttle - pitchCtrl - rollCtrl + yawCtrl;

    // clamp / scale 保证 0..100
    float maxv = fmaxf(fmaxf(m0,m1), fmaxf(m2,m3));
    float minv = fminf(fminf(m0,m1), fminf(m2,m3));
    const float OUT_MIN = 0.0f, OUT_MAX = 100.0f;
    if (maxv > OUT_MAX || minv < OUT_MIN) {
        float scale = 1.0f;
        if (maxv - minv > 0.0f) scale = fminf(OUT_MAX / maxv, OUT_MIN / minv);
        m0 *= scale; m1 *= scale; m2 *= scale; m3 *= scale;
    }

    // 写入 PWM 缓冲（短临界区）
    pwmDutyBuffer[0] = Map_Percent_To_Real(m0);
    pwmDutyBuffer[1] = Map_Percent_To_Real(m1);
    pwmDutyBuffer[2] = Map_Percent_To_Real(m2);
    pwmDutyBuffer[3] = Map_Percent_To_Real(m3);
}