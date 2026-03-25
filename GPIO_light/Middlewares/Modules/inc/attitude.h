#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"
#include "ekf_state7.h"
#include "lowpass.h"
#include "spatial_math.h"
#include "calibrate.h"
#include "persistence.h"

extern uint8_t imu_rx_buf[14];
extern uint8_t mag_rx_buf[6];
extern float altitude_rx;
extern float temperature_rx;
// extern uint8_t bmp_rx_buf[6]

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale);
void Attitude_Update(float dt);
void Attitude_IsStill(uint8_t *still);
void Attitude_GetEuler(float *yaw, float *pitch, float *roll);
void Attitude_GetQuat(sm_quat_t q);
void Attitude_GetGyro(sm_vec3_t gyro);
void Attitude_GetAccel(sm_vec3_t accel);
// void Attitude_GetMag(sm_vec3_t mag);
void Attitude_GetAltitude(float *altitude);

void Attitude_Calibrate(void);
void Attitude_CalibratingFace(uint8_t *face);


#endif /* __ATTITUDE_H */