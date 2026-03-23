#include "state_estimator.h"
#include <string.h>

/**
 * @brief 初始化姿态估计器
 */
void Estimator_Attitude_Init(Estimator_Attitude_EKF_t *est, const sm_vec3_t mag_ref) {
    memset(est, 0, sizeof(*est));
    est->q[0] = 1.0f;
    est->q_corr[0] = 1.0f;
    est->gyro_bias[0] = est->gyro_bias[1] = est->gyro_bias[2] = 0.0f;
    est->gyro_corr[0] = est->gyro_corr[1] = est->gyro_corr[2] = 0.0f;
    est->mag_ref[0] = mag_ref[0]; est->mag_ref[1] = mag_ref[1]; est->mag_ref[2] = mag_ref[2];

    est->Q_angle = 0.001f;
    est->Q_gyro = 0.003f;
    est->R_accel = 0.1f;
    est->R_mag = 0.1f;

    est->Kp = 2.0f;
    est->Ki = 0.001f;

    arm_mat_init_f32(&est->P, 7, 7, est->P_data);
    memset(est->P_data, 0, sizeof(est->P_data));
    for (int i = 0; i < 7; i++) est->P_data[i*7 + i] = 0.1f;
    LowPass_Filter_Init(&est->magYawFilt, 0.0f, 0.0f);
}

/**
 * @brief 姿态更新 (融合加速度计与磁力计的互补/EKF 混合逻辑)
 */
void Estimator_Attitude_Update(Estimator_Attitude_EKF_t *est, 
                               const sm_vec3_t gyro, 
                               const sm_vec3_t accel, 
                               const sm_vec3_t mag, 
                               float dt) {
    
    if (dt <= 0) return;

    sm_vec3_t accel_n; memcpy(accel_n, accel, sizeof(accel_n));
    sm_vec3_t mag_n;   memcpy(mag_n, mag, sizeof(mag_n));
    Spatial_Vec3Normalize(accel_n);
    Spatial_Vec3Normalize(mag_n);

    sm_vec3_t omega = {gyro[0] - est->gyro_bias[0],
                       gyro[1] - est->gyro_bias[1],
                       gyro[2] - est->gyro_bias[2]};

    sm_quat_t q0 = {est->q[0], est->q[1], est->q[2], est->q[3]};
    sm_quat_t qdot;
    qdot[0] = 0.0f;
    qdot[1] = 0.5f * ( omega[0]*q0[0] + omega[1]*q0[3] - omega[2]*q0[2]);
    qdot[2] = 0.5f * ( omega[1]*q0[0] - omega[0]*q0[3] + omega[2]*q0[1]);
    qdot[3] = 0.5f * ( omega[2]*q0[0] + omega[0]*q0[2] - omega[1]*q0[1]);

    est->q[0] = q0[0] + qdot[0] * dt;
    est->q[1] = q0[1] + qdot[1] * dt;
    est->q[2] = q0[2] + qdot[2] * dt;
    est->q[3] = q0[3] + qdot[3] * dt;
    Spatial_QuatNormalize(est->q);

    float F_data[49] = {0};
    for (int i = 0; i < 7; i++) F_data[i*7 + i] = 1.0f;
    arm_matrix_instance_f32 F;
    arm_mat_init_f32(&F, 7, 7, F_data);
    arm_matrix_instance_f32 P_temp;
    float P_temp_buf[49];
    arm_mat_init_f32(&P_temp, 7, 7, P_temp_buf);
    arm_mat_mult_f32(&F, &est->P, &P_temp);
    arm_mat_mult_f32(&P_temp, &F, &est->P);
    for (int i = 0; i < 7; i++) est->P_data[i*7+i] += (i<4 ? est->Q_angle : est->Q_gyro) * dt;

    sm_vec3_t g_b, m_b;
    sm_vec3_t ref_g = {0.0f, 0.0f, 1.0f};
    sm_vec3_t ref_m = {est->mag_ref[0], est->mag_ref[1], est->mag_ref[2]};
    Spatial_RotatePointByQuat(g_b, ref_g, est->q);
    Spatial_RotatePointByQuat(m_b, ref_m, est->q);

    sm_vec3_t err_acc, err_mag, total_err;
    Spatial_Vec3Cross(err_acc, accel_n, g_b);
    Spatial_Vec3Cross(err_mag, mag_n, m_b);
    total_err[0] = err_acc[0] + err_mag[0];
    total_err[1] = err_acc[1] + err_mag[1];
    total_err[2] = err_acc[2] + err_mag[2];

    est->Kp = 2.0f;
    est->Ki = 0.001f;
    est->gyro_corr[0] = omega[0] + est->Kp * total_err[0];
    est->gyro_corr[1] = omega[1] + est->Kp * total_err[1];
    est->gyro_corr[2] = omega[2] + est->Kp * total_err[2];
    est->gyro_bias[0] += est->Ki * total_err[0] * dt;
    est->gyro_bias[1] += est->Ki * total_err[1] * dt;
    est->gyro_bias[2] += est->Ki * total_err[2] * dt;

    sm_quat_t q1 = {est->q[0], est->q[1], est->q[2], est->q[3]};
    sm_quat_t qdot2;
    qdot2[0] = 0.0f;
    qdot2[1] = 0.5f * (est->gyro_corr[0]*q1[0] + est->gyro_corr[1]*q1[3] - est->gyro_corr[2]*q1[2]);
    qdot2[2] = 0.5f * (est->gyro_corr[1]*q1[0] - est->gyro_corr[0]*q1[3] + est->gyro_corr[2]*q1[1]);
    qdot2[3] = 0.5f * (est->gyro_corr[2]*q1[0] + est->gyro_corr[0]*q1[2] - est->gyro_corr[1]*q1[1]);

    est->q[0] = q1[0] + qdot2[0] * dt;
    est->q[1] = q1[1] + qdot2[1] * dt;
    est->q[2] = q1[2] + qdot2[2] * dt;
    est->q[3] = q1[3] + qdot2[3] * dt;
    Spatial_QuatNormalize(est->q);

    memcpy(est->q_corr, est->q, sizeof(est->q));
}

/**
 * @brief 初始化高度估计器
 */
void Estimator_Altitude_Init(Estimator_Altitude_EKF_t *est) {
    est->height = 0.0f;
    est->vz = 0.0f;
    est->accel_bias = 0.0f;

    est->Q_height = 0.1f;
    est->Q_accel = 0.1f;
    est->R_baro = 1.0f;

    arm_mat_init_f32(&est->P, 3, 3, est->P_data);
    memset(est->P_data, 0, sizeof(est->P_data));
    for(int i=0; i<3; i++) est->P_data[i*3 + i] = 1.0f; 
}

/**
 * @brief 高度融合更新 (气压计 + 加速度计)
 */
void Estimator_Altitude_Update(Estimator_Altitude_EKF_t *est, 
                               float accel_z, 
                               float baro_alt, 
                               float dt) {
    // 1. 预测步 (物理方程)
    float acc_eff = accel_z - est->accel_bias;
    est->height += est->vz * dt + 0.5f * acc_eff * dt * dt;
    est->vz     += acc_eff * dt;

    // 2. 协方差矩阵预测 (使用 CMSIS-DSP 处理矩阵)
    arm_matrix_instance_f32 F, Ft, Q, P_temp;
    float32_t F_data[9] = {1, dt, 0, 0, 1, dt, 0, 0, 1};
    float32_t Ft_data[9], Q_data[9] = {0}, P_buf[9];
    
    arm_mat_init_f32(&F, 3, 3, F_data);
    arm_mat_init_f32(&Ft, 3, 3, Ft_data);
    arm_mat_init_f32(&P_temp, 3, 3, P_buf);
    
    arm_mat_trans_f32(&F, &Ft);
    arm_mat_mult_f32(&F, &est->P, &P_temp);
    arm_mat_mult_f32(&P_temp, &Ft, &est->P);
    
    // 增加过程噪声
    est->P_data[0] += est->Q_height;
    est->P_data[4] += est->Q_accel;
    est->P_data[8] += 0.0001f;

    // 3. 测量更新 (气压计)
    float S = est->P_data[0] + est->R_baro;
    float K[3] = { est->P_data[0]/S, est->P_data[3]/S, est->P_data[6]/S };
    
    float innov = baro_alt - est->height;
    est->height     += K[0] * innov;
    est->vz         += K[1] * innov;
    est->accel_bias += K[2] * innov;

    // 4. 更新协方差 P = (I - KH)P
    float32_t p0 = est->P_data[0], p3 = est->P_data[3], p6 = est->P_data[6];
    est->P_data[0] -= K[0] * p0; est->P_data[1] -= K[0] * est->P_data[1]; est->P_data[2] -= K[0] * est->P_data[2];
    est->P_data[3] -= K[1] * p0; est->P_data[4] -= K[1] * est->P_data[1]; est->P_data[5] -= K[1] * est->P_data[2];
    est->P_data[6] -= K[2] * p0; est->P_data[7] -= K[2] * est->P_data[1]; est->P_data[8] -= K[2] * est->P_data[2];
}