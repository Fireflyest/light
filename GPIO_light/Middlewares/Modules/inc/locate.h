#ifndef __LOCATE_H
#define __LOCATE_H

#include "kalman.h"
#include "math3d.h"
#include "lowpass.h"

#define TEMP_TAU  5.0f    // 温度很慢，5s 起
#define PRESS_TAU 2.0f    // 气压较慢，2s 起
#define ALT_TAU  1.0f    // 高度滤波时间常数
#define AZ_TAU   0.1f    // 加速度滤波时间常数

extern LowPass_Filter_t tempFilt, pressFilt, altFilt, azFilt;
extern Locate_Kalman_EKF_t loc_ekf;

extern float temperature;
extern float barometricPressure;
extern float altitude;
extern float gravity_est[3];

void Locate_Init();

void Locate_Update(float dt, float32_t q[4]);

#endif /* __LOCATE_H */