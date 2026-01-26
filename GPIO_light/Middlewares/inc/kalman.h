#ifndef __KALMAN_H
#define __KALMAN_H

#include "arm_math.h"

typedef struct {
    float32_t q[4];      // 状态向量：四元数 [q0, q1, q2, q3]
    float32_t bias[3];   // 陀螺仪零偏 [bx, by, bz]
    
    // 协方差矩阵使用 arm_matrix 结构
    float32_t P_data[49]; 
    arm_matrix_instance_f32 P;
    
    float32_t Q_angle;
    float32_t Q_gyro; 
    float32_t R_accel;
} Kalman_EKF_t;

/**
 * @brief 初始化卡尔曼结构体
 */
void Kalman_Init(Kalman_EKF_t *ekf);


void Kalman_Update(Kalman_EKF_t *ekf, float32_t gx, float32_t gy, float32_t gz, 
                  float32_t ax, float32_t ay, float32_t az, 
                  float32_t mx, float32_t my, float32_t mz, float32_t dt);

#endif /* __KALMAN_H */