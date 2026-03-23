#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H

#include "arm_math.h"
#include "spatial_math.h"
#include "lowpass.h"

/**
 * @brief 姿态估计 EKF 结构体 (基于四元数)
 */
typedef struct {
    sm_quat_t q;                 // 状态向量：四元数 [q0, q1, q2, q3]
    sm_quat_t q_corr;            // 校正后的四元数输出
    sm_vec3_t gyro_bias;         // 陀螺仪零偏 [bx, by, bz]
    sm_vec3_t gyro_corr;         // 校正后的角速度 (减去 bias)
    sm_vec3_t mag_ref;           // 磁力计参考向量 (地磁场方向)
    
    float32_t P_data[49];        // 7x7 矩阵
    arm_matrix_instance_f32 P;
    
    float32_t Q_angle;           // 过程噪声：角度
    float32_t Q_gyro;            // 过程噪声：陀螺仪
    float32_t R_accel;           // 观测噪声：加速度计
    float32_t R_mag;             // 观测噪声：磁力计

    float32_t Kp, Ki;            // 用于互补滤波或初始化权重

    LowPass_Filter_t magYawFilt; // 磁力计航向低通滤波
} Estimator_Attitude_EKF_t;

/**
 * @brief 垂直导航/高度估计卡尔曼结构体
 */
typedef struct {
    float32_t height;        // 状态量：高度 (z)
    float32_t vz;            // 状态量：垂直速度
    float32_t accel_bias;    // 垂直加速度计偏置

    // 3x3 协方差矩阵
    float32_t P_data[9];
    arm_matrix_instance_f32 P;

    float32_t Q_height;      // 过程噪声
    float32_t Q_accel;       // 过程噪声
    float32_t R_baro;        // 观测噪声：气压计
} Estimator_Altitude_EKF_t;



/**
 * @brief 初始化姿态估计算法
 */
void Estimator_Attitude_Init(Estimator_Attitude_EKF_t *est, const sm_vec3_t mag_ref);

/**
 * @brief 更新姿态估计 (融合 9 轴 IMU 数据)
 */
void Estimator_Attitude_Update(Estimator_Attitude_EKF_t *est, 
                               const sm_vec3_t gyro, 
                               const sm_vec3_t accel, 
                               const sm_vec3_t mag, 
                               float dt);

/**
 * @brief 初始化高度估计算法
 */
void Estimator_Altitude_Init(Estimator_Altitude_EKF_t *est);

/**
 * @brief 使用加速度计和气压计更新高度信息
 */
void Estimator_Altitude_Update(Estimator_Altitude_EKF_t *est, 
                               float accel_z, 
                               float baro_alt, 
                               float dt);

#endif /* __STATE_ESTIMATOR_H */