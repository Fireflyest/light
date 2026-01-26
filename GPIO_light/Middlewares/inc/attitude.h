#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "arm_math.h"
#include "kalman.h"

extern float gyro_offset[3];

typedef struct {
    float pitch;
    float roll;
    float yaw;
} Attitude_t;

extern Attitude_t imu_attitude;
extern Kalman_EKF_t imu_ekf; // 暴露 EKF 实例

void Attitude_Update(float dt);

#endif /* __ATTITUDE_H */