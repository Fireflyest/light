#include "kalman.h"

void Attitude_Kalman_Init(Attitude_Kalman_EKF_t *ekf) {
    ekf->q[0] = 1.0f; ekf->q[1] = 0.0f; ekf->q[2] = 0.0f; ekf->q[3] = 0.0f;
    ekf->bias[0] = 0.0f; ekf->bias[1] = 0.0f; ekf->bias[2] = 0.0f;
    
    ekf->Q_angle = 0.001f;
    ekf->Q_gyro = 0.003f;
    ekf->R_accel = 0.1f;

    arm_mat_init_f32(&ekf->P, 7, 7, ekf->P_data);
    for(int i=0; i<49; i++) ekf->P_data[i] = 0.0f;
    for(int i=0; i<7; i++) ekf->P_data[i*7 + i] = 0.1f; // 对角线初始化
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
    
    // 1. 归一化加速度计
    float32_t norm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    // 2. 估计重力方向 (机体坐标系下)
    float32_t vx = 2.0f * (q1*q3 - q0*q2);
    float32_t vy = 2.0f * (q0*q1 + q2*q3);
    float32_t vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 计算加速度误差（外积）- 用于纠正 Pitch/Roll
    float32_t ex = (ay * vz - az * vy);
    float32_t ey = (az * vx - ax * vz);
    float32_t ez = (ax * vy - ay * vx);

    // 4. 重要：磁力计纠正 Yaw (解决绕 Z 轴旋转问题)
    // 如果不使用磁力计，ez 此时只能由重力纠正，而重力对 Z 轴旋转不降噪
    norm = 1.0f / sqrtf(mx*mx + my*my + mz*mz);
    mx *= norm; my *= norm; mz *= norm;
    float32_t hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + my * 2.0f * (q1*q2 - q0*q3) + mz * 2.0f * (q1*q3 + q0*q2);
    float32_t hy = mx * 2.0f * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz * 2.0f * (q2*q3 - q0*q1);
    float32_t bx = sqrtf(hx*hx + hy*hy);
    float32_t bz = mx * 2.0f * (q1*q3 - q0*q2) + my * 2.0f * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    float32_t wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
    float32_t wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q0*q1 + q2*q3);
    float32_t wz = bx * 2.0f * (q0*q2 + q1*q3) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    ez += (mx * wy - my * wx); // 此时 ez 包含了来自磁力计的航向偏差

    // 5. 修正 Bias 并更新四元数
    float32_t Kp = 2.0f; // 比例增益：纠正速度
    float32_t Ki = 0.001f; // 积分增益：消除 bias (不可太大，否则加速旋转)

    ekf->bias[0] -= Ki * ex;
    ekf->bias[1] -= Ki * ey;
    ekf->bias[2] -= Ki * ez;

    gx = gx - ekf->bias[0] + Kp * ex;
    gy = gy - ekf->bias[1] + Kp * ey;
    gz = gz - ekf->bias[2] + Kp * ez;

    // 更新四元数 (一阶龙格库塔)
    ekf->q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    ekf->q[1] += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    ekf->q[2] += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    ekf->q[3] += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;

    // 必须归一化
    norm = 1.0f / sqrtf(ekf->q[0]*ekf->q[0] + ekf->q[1]*ekf->q[1] + ekf->q[2]*ekf->q[2] + ekf->q[3]*ekf->q[3]);
    ekf->q[0] *= norm; ekf->q[1] *= norm; ekf->q[2] *= norm; ekf->q[3] *= norm;
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

    // 协方差预测
    float P00 = ekf->P_data[0] + dt * (ekf->P_data[3] + ekf->P_data[1]) + dt * dt * ekf->P_data[4] + ekf->Q_height;
    float P01 = ekf->P_data[1] + dt * ekf->P_data[4];
    float P02 = ekf->P_data[2];
    float P10 = ekf->P_data[3] + dt * ekf->P_data[4];
    float P11 = ekf->P_data[4] + ekf->Q_accel;
    float P12 = ekf->P_data[5];
    float P20 = ekf->P_data[6];
    float P21 = ekf->P_data[7];
    float P22 = ekf->P_data[8] + 1e-6f; // bias噪声极小

    ekf->P_data[0] = P00;
    ekf->P_data[1] = P01;
    ekf->P_data[2] = P02;
    ekf->P_data[3] = P10;
    ekf->P_data[4] = P11;
    ekf->P_data[5] = P12;
    ekf->P_data[6] = P20;
    ekf->P_data[7] = P21;
    ekf->P_data[8] = P22;

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

    // 协方差更新
    float P00_ = ekf->P_data[0], P01_ = ekf->P_data[1], P02_ = ekf->P_data[2];
    float P10_ = ekf->P_data[3], P11_ = ekf->P_data[4], P12_ = ekf->P_data[5];
    float P20_ = ekf->P_data[6], P21_ = ekf->P_data[7], P22_ = ekf->P_data[8];

    ekf->P_data[0] -= K0 * P00_;
    ekf->P_data[1] -= K0 * P01_;
    ekf->P_data[2] -= K0 * P02_;
    ekf->P_data[3] -= K1 * P00_;
    ekf->P_data[4] -= K1 * P01_;
    ekf->P_data[5] -= K1 * P02_;
    ekf->P_data[6] -= K2 * P00_;
    ekf->P_data[7] -= K2 * P01_;
    ekf->P_data[8] -= K2 * P02_;
}