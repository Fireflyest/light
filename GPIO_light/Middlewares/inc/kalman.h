#ifndef __KALMAN_H
#define __KALMAN_H

#include "arm_math.h"

typedef struct {
    float32_t q[4];      // 状态向量：四元数 [q0, q1, q2, q3]
    float32_t bias[3];   // 陀螺仪零偏 [bx, by, bz]

    float32_t q_corr[4];  // 校正后的四元数
    float32_t gyro_corr[3]; // 校正后的角速度
    
    // 协方差矩阵使用 arm_matrix 结构
    float32_t P_data[49]; 
    arm_matrix_instance_f32 P;
    
    float32_t Q_angle;
    float32_t Q_gyro; 
    float32_t R_accel;
} Attitude_Kalman_EKF_t;

typedef struct
{
    float32_t h;          // 状态量：高度
    float32_t vz;         // 状态量：垂直速度
    float32_t bz;         // 垂直加速度计偏置

    // 协方差矩阵使用 arm_matrix 结构
    float32_t P_data[9];
    arm_matrix_instance_f32 P;

    float32_t Q_height;
    float32_t Q_accel;
    float32_t R_pressure;
} Locate_Kalman_EKF_t;


/**
 * @brief 初始化卡尔曼结构体
 */
void Attitude_Kalman_Init(Attitude_Kalman_EKF_t *ekf);

void Locate_Kalman_Init(Locate_Kalman_EKF_t *ekf);

void Attitude_Kalman_Update(Attitude_Kalman_EKF_t *ekf, float32_t gx, float32_t gy, float32_t gz, 
                  float32_t ax, float32_t ay, float32_t az, 
                  float32_t mx, float32_t my, float32_t mz, float32_t dt);

/**
 * @brief 使用加速度计和气压计更新高度卡尔曼滤波器
 * 当前只有高度一个观测量
 */
void Locate_Kalman_Update(Locate_Kalman_EKF_t *ekf, float32_t az, float32_t altitude, float32_t dt);

#endif /* __KALMAN_H */