#include "kalman.h"

void Kalman_Init(Kalman_EKF_t *ekf) {
    ekf->q[0] = 1.0f; ekf->q[1] = 0.0f; ekf->q[2] = 0.0f; ekf->q[3] = 0.0f;
    ekf->bias[0] = 0.0f; ekf->bias[1] = 0.0f; ekf->bias[2] = 0.0f;
    
    ekf->Q_angle = 0.001f;
    ekf->Q_gyro = 0.003f;
    ekf->R_accel = 0.1f;

    arm_mat_init_f32(&ekf->P, 7, 7, ekf->P_data);
    for(int i=0; i<49; i++) ekf->P_data[i] = 0.0f;
    for(int i=0; i<7; i++) ekf->P_data[i*7 + i] = 0.1f; // 对角线初始化
}

void Kalman_Update(Kalman_EKF_t *ekf, float32_t gx, float32_t gy, float32_t gz, 
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