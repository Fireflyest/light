#include "kalman.h"

void Attitude_Kalman_Init(Attitude_Kalman_EKF_t *ekf) {
    ekf->q[0] = 1.0f; ekf->q[1] = 0.0f; ekf->q[2] = 0.0f; ekf->q[3] = 0.0f;
    ekf->bias[0] = 0.0f; ekf->bias[1] = 0.0f; ekf->bias[2] = 0.0f;
    ekf->gyro_corr[0] = 0.0f; ekf->gyro_corr[1] = 0.0f; ekf->gyro_corr[2] = 0.0f;
    
    ekf->Q_angle = 0.001f;
    ekf->Q_gyro = 0.003f;
    ekf->R_accel = 0.1f;
    ekf->Kp = 2.0f;
    ekf->Ki = 0.001f;

    arm_mat_init_f32(&ekf->P, 7, 7, ekf->P_data);
    for(int i=0; i<49; i++) ekf->P_data[i] = 0.0f;
    for(int i=0; i<7; i++) ekf->P_data[i*7 + i] = 0.1f; // 对角线初始化

    LowPass_Filter_Init(&ekf->magYawFilt, 0.0f, 0.0f);
}

void Locate_Kalman_Init(Locate_Kalman_EKF_t *ekf) {
    ekf->h = 0.0f;
    ekf->vz = 0.0f;
    ekf->bz = 0.0f;

    ekf->Q_height = 0.1f;
    ekf->Q_accel = 0.1f;
    ekf->R_pressure = 1.0f;

    arm_mat_init_f32(&ekf->P, 3, 3, ekf->P_data);
    for(int i=0; i<9; i++) ekf->P_data[i] = 0.0f;
    for(int i=0; i<3; i++) ekf->P_data[i*3 + i] = 1.0f; // 对角线初始化
}

void Attitude_Kalman_Update(Attitude_Kalman_EKF_t *ekf, float32_t gx, float32_t gy, float32_t gz, 
                  float32_t ax, float32_t ay, float32_t az, 
                  float32_t mx, float32_t my, float32_t mz, float32_t dt) {
    float32_t q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];
    float32_t integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 积分项（可选）

    // 1. 归一化加速度和磁力计（使用 DSP 向量运算）
    float32_t a_vec[3] = {ax, ay, az};
    float32_t m_vec[3] = {mx, my, mz};
    float32_t norm_a_sq = ax*ax + ay*ay + az*az;
    float32_t norm_m_sq = mx*mx + my*my + mz*mz;
    float32_t norm_a, norm_m;
    arm_sqrt_f32(norm_a_sq, &norm_a);  // DSP: 平方根
    arm_sqrt_f32(norm_m_sq, &norm_m);
    arm_scale_f32(a_vec, 1.0f / norm_a, a_vec, 3);  // DSP: 向量缩放
    arm_scale_f32(m_vec, 1.0f / norm_m, m_vec, 3);
    ax = a_vec[0]; ay = a_vec[1]; az = a_vec[2];
    mx = m_vec[0]; my = m_vec[1]; mz = m_vec[2];

    // 2. 计算重力方向（机体坐标系）
    float32_t vx = 2.0f * (q1*q3 - q0*q2);
    float32_t vy = 2.0f * (q0*q1 + q2*q3);
    float32_t vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 计算磁场方向（机体坐标系，假设水平磁场）
    float32_t hx = 2.0f * mx * (0.5f - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 - q0*q3) + 2.0f * mz * (q1*q3 + q0*q2);
    float32_t hy = 2.0f * mx * (q1*q2 + q0*q3) + 2.0f * my * (0.5f - q1*q1 - q3*q3) + 2.0f * mz * (q2*q3 - q0*q1);
    float32_t hz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + 2.0f * mz * (0.5f - q1*q1 - q2*q2);
    float32_t bx_sq = hx*hx + hy*hy;
    float32_t bz = hz;
    float32_t bx;
    arm_sqrt_f32(bx_sq, &bx);  // DSP: 平方根

    // 4. 计算误差（参考方向与测量方向的叉积）
    // 加速度误差
    float32_t ex = ay * vz - az * vy;
    float32_t ey = az * vx - ax * vz;
    float32_t ez = ax * vy - ay * vx;
    // 磁力计误差（只用水平分量）
    float32_t exm = my * bz - mz * hy;
    float32_t eym = mz * hx - mx * bz;
    float32_t ezm = mx * hy - my * hx;

    // 5. PI 控制器（积分项可选）
    if (ekf->Ki > 0.0f) {
        integralFBx += ekf->Ki * ex * dt;
        integralFBy += ekf->Ki * ey * dt;
        integralFBz += ekf->Ki * ezm * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // 6. 应用比例校正
    gx += ekf->Kp * ex;
    gy += ekf->Kp * ey;
    gz += ekf->Kp * ezm;

    // 7. 积分四元数（一阶）
    float half_dt = 0.5f * dt;
    q0 += half_dt * (-q1 * gx - q2 * gy - q3 * gz);
    q1 += half_dt * ( q0 * gx + q2 * gz - q3 * gy);
    q2 += half_dt * ( q0 * gy - q1 * gz + q3 * gx);
    q3 += half_dt * ( q0 * gz + q1 * gy - q2 * gx);

    // 8. 归一化（使用 DSP 四元数归一化）
    float32_t q_vec[4] = {q0, q1, q2, q3};
    arm_quaternion_normalize_f32(q_vec, q_vec, 1);  // DSP: 四元数归一化
    ekf->q[0] = q_vec[0];
    ekf->q[1] = q_vec[1];
    ekf->q[2] = q_vec[2];
    ekf->q[3] = q_vec[3];

    // 9. 更新 gyro_corr（可选，用于外部使用）
    ekf->gyro_corr[0] = gx;
    ekf->gyro_corr[1] = gy;
    ekf->gyro_corr[2] = gz;
}


void Locate_Kalman_Update(Locate_Kalman_EKF_t *ekf, float32_t az, float32_t altitude, float32_t dt) {
    // 状态变量: ekf->h (高度), ekf->vz (垂直速度), ekf->bz (加速度零偏)
    // az: 加速度计Z轴（去重力后，单位 m/s^2）
    // pressure: 气压计高度（单位 m）
    // dt: 时间间隔（单位 s）

    // 1. 预测（时间更新）
    float acc = az - ekf->bz;
    ekf->h  += ekf->vz * dt + 0.5f * acc * dt * dt;
    ekf->vz += acc * dt;

    // 协方差预测（使用 DSP 矩阵运算）
    arm_matrix_instance_f32 F, Ft, Q, P_pred, temp;
    float32_t F_data[9] = {1, dt, 0, 0, 1, dt, 0, 0, 1};
    float32_t Ft_data[9];
    float32_t temp_data[9];  // 添加 temp 数据数组
    float32_t Q_data[9] = {ekf->Q_height, 0, 0, 0, ekf->Q_accel, 0, 0, 0, 1e-6f};
    arm_mat_init_f32(&F, 3, 3, F_data);
    arm_mat_init_f32(&Ft, 3, 3, Ft_data);
    arm_mat_init_f32(&temp, 3, 3, temp_data);  // 修复：初始化 temp
    arm_mat_init_f32(&Q, 3, 3, Q_data);
    arm_mat_init_f32(&P_pred, 3, 3, ekf->P_data);
    arm_mat_trans_f32(&F, &Ft);
    arm_mat_mult_f32(&F, &ekf->P, &temp);  // 现在正常
    arm_mat_mult_f32(&temp, &Ft, &P_pred);
    arm_mat_add_f32(&P_pred, &Q, &ekf->P);

    // 2. 更新（测量更新，气压计高度）
    // 观测矩阵 H = [1 0 0]
    float S = ekf->P_data[0] + ekf->R_pressure;
    float K0 = ekf->P_data[0] / S;
    float K1 = ekf->P_data[3] / S;
    float K2 = ekf->P_data[6] / S;

    float y = altitude - ekf->h; // 高度残差

    ekf->h  += K0 * y;
    ekf->vz += K1 * y;
    ekf->bz += K2 * y;

    // 协方差更新（使用 DSP 矩阵运算）
    arm_matrix_instance_f32 I, KH;
    float32_t I_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float32_t KH_data[9] = {K0, 0, 0, K1, 0, 0, K2, 0, 0};
    arm_mat_init_f32(&I, 3, 3, I_data);
    arm_mat_init_f32(&KH, 3, 3, KH_data);
    arm_mat_sub_f32(&I, &KH, &temp);  // 复用 temp
    arm_mat_mult_f32(&temp, &ekf->P, &ekf->P);
}