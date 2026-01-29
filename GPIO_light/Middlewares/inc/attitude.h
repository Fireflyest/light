#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "kalman.h"
#include "math3d.h"

extern Attitude_Kalman_EKF_t imu_ekf; // 暴露 EKF 实例

void Attitude_Update(float dt, Quaternion* q_bias);

#endif /* __ATTITUDE_H */