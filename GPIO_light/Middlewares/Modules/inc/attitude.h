#ifndef __ATTITUDE_H
#define __ATTITUDE_H

// #include "state_estimator.h"
// #include "spatial_math.h"

// extern Estimator_Attitude_EKF_t imu_ekf;
#include "stm32f4xx.h"
#include "ekf_state6.h"
#include "lowpass.h"
#include "spatial_math.h"
#include "calibrate.h"

extern uint8_t imu_rx_buf[14];
extern uint8_t mag_rx_buf[6];
// extern uint8_t bmp_rx_buf[6]

extern sm_quat_t last_quat;
extern LowPass_Filter_t is_still;
extern uint8_t accel_face;


extern EKF_Handle_t imu_ekf;

void Attitude_Init(void);
void Attitude_Update(float dt);
void Attitude_IsStill(uint8_t *still);

#endif /* __ATTITUDE_H */