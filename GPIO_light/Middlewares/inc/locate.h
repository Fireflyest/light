#ifndef __LOCATE_H
#define __LOCATE_H

#include "kalman.h"
#include "math3d.h"

extern Locate_Kalman_EKF_t loc_ekf;

extern float temperature;
extern float barometricPressure;
extern float altitude;


void Locate_Update(float dt, float32_t q[4]);

#endif /* __LOCATE_H */