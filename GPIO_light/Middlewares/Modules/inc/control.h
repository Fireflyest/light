#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "pid.h"
#include "lowpass.h"
#include "spatial_math.h"

extern PID_t pidRoll, pidPitch, pidYaw, pidHeight;
extern PID_t pidRateRoll, pidRatePitch, pidRateYaw;
extern float baseThrottle;
extern __IO float rateSetRoll, rateSetPitch, rateSetYaw;
extern __IO float thrustOutput;


#define GYRO_TAU    0.004f
#define RATE_LOOP_HZ 1000
#define GYRO_SENS_DEFAULT 16.4f // LSB per dps（根据你的 IMU 改）
#define DEG2RAD (3.14159265358979323846f/180.0f)
#define RAD2DEG 57.29577951308232f

void RateControl_Init(uint32_t freq);
void RateControl_Loop(void);
void RateControl_TargetAttitude(sm_quat_t q_target, float h_target);

#endif /* __CONTROL_H */