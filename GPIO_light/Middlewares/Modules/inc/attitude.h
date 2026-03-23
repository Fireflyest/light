#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "state_estimator.h"
#include "spatial_math.h"

extern Estimator_Attitude_EKF_t imu_ekf;

void Attitude_Init(void);
void Attitude_Update(float dt);

#endif /* __ATTITUDE_H */