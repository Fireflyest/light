#include "ekf_state7.h"

#include <string.h>
#include <math.h>
#include "arm_math.h"

#define GRAVITY 9.81f

// ==================================
// 四元数运算
// ==================================
static void Quaternion_Multiply(const float32_t q1[4], const float32_t q2[4], float32_t q_out[4])
{
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void Quaternion_Normalize(float32_t q[4])
{
    float32_t norm;
    arm_status status = arm_sqrt_f32(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3], &norm);
    if (status == ARM_MATH_SUCCESS && norm > 1e-6f) {
        float32_t inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

// ==================================
// 状态转移 (只更新状态, 不用作 Jacobian 输入)
// ==================================
static void State_Transition(EKF_Handle_t *ekf, const float32_t gyro[3],
                              const float32_t accel[3], float32_t dt)
{
    float32_t *x = ekf->x;

    // 角速度 = 陀螺仪 - 偏置
    float32_t omega[3] = {
        gyro[0] - x[EKF_IDX_BX],
        gyro[1] - x[EKF_IDX_BY],
        gyro[2] - x[EKF_IDX_BZ]
    };

    // 四元数更新
    float32_t omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float32_t q_dot[4];
    Quaternion_Multiply(x, omega_q, q_dot);
    arm_scale_f32(q_dot, 0.5f * dt, q_dot, 4);
    arm_add_f32(x, q_dot, x, 4);
    Quaternion_Normalize(x);

    // 世界坐标系垂直加速度
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float32_t R20 = 2.0f * (q0*q2 + q1*q3);
    float32_t R21 = 2.0f * (q2*q3 - q0*q1);
    float32_t R22 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float32_t az_world = R20 * accel[0] + R21 * accel[1] + R22 * accel[2] - GRAVITY;

    // 垂直速度和高度
    x[EKF_IDX_VZ]  += az_world * dt;
    x[EKF_IDX_ALT] += x[EKF_IDX_VZ] * dt;
}

// ==================================
// Jacobian F (用旧四元数计算)
// ==================================
static void Compute_JacobianF(EKF_Handle_t *ekf, const float32_t gyro[3],
                               const float32_t accel[3], float32_t dt,
                               const float32_t q_old[4])
{
    // 用旧四元数 q(k) 计算, 不是预测后的 q(k+1)
    float32_t q0 = q_old[0], q1 = q_old[1], q2 = q_old[2], q3 = q_old[3];
    float32_t wx = gyro[0] - ekf->x[EKF_IDX_BX];
    float32_t wy = gyro[1] - ekf->x[EKF_IDX_BY];
    float32_t wz = gyro[2] - ekf->x[EKF_IDX_BZ];
    float32_t ax = accel[0], ay = accel[1], az = accel[2];
    float32_t *F = ekf->F_data;

    // 初始化为单位矩阵
    memset(F, 0, EKF_STATE_DIM * EKF_STATE_DIM * sizeof(float32_t));
    for (int i = 0; i < EKF_STATE_DIM; i++)
        F[i * EKF_STATE_DIM + i] = 1.0f;

    float32_t half_dt = 0.5f * dt;

    // ∂q/∂q
    F[0*EKF_STATE_DIM+1] = -half_dt * wx;
    F[0*EKF_STATE_DIM+2] = -half_dt * wy;
    F[0*EKF_STATE_DIM+3] = -half_dt * wz;
    F[1*EKF_STATE_DIM+0] =  half_dt * wx;
    F[1*EKF_STATE_DIM+2] =  half_dt * wz;
    F[1*EKF_STATE_DIM+3] = -half_dt * wy;
    F[2*EKF_STATE_DIM+0] =  half_dt * wy;
    F[2*EKF_STATE_DIM+1] = -half_dt * wz;
    F[2*EKF_STATE_DIM+3] =  half_dt * wx;
    F[3*EKF_STATE_DIM+0] =  half_dt * wz;
    F[3*EKF_STATE_DIM+1] =  half_dt * wy;
    F[3*EKF_STATE_DIM+2] = -half_dt * wx;

    // ∂q/∂bias
    F[0*EKF_STATE_DIM+4] =  half_dt * q1;
    F[0*EKF_STATE_DIM+5] =  half_dt * q2;
    F[0*EKF_STATE_DIM+6] =  half_dt * q3;
    F[1*EKF_STATE_DIM+4] = -half_dt * q0;
    F[1*EKF_STATE_DIM+5] =  half_dt * q3;
    F[1*EKF_STATE_DIM+6] = -half_dt * q2;
    F[2*EKF_STATE_DIM+4] = -half_dt * q3;
    F[2*EKF_STATE_DIM+5] = -half_dt * q0;
    F[2*EKF_STATE_DIM+6] =  half_dt * q1;
    F[3*EKF_STATE_DIM+4] =  half_dt * q2;
    F[3*EKF_STATE_DIM+5] = -half_dt * q1;
    F[3*EKF_STATE_DIM+6] = -half_dt * q0;

    // ∂h/∂vz = dt
    F[7*EKF_STATE_DIM+8] = dt;

    // ∂vz/∂q (用旧四元数)
    F[8*EKF_STATE_DIM+0] = dt * ( 2.0f*q2*ax - 2.0f*q1*ay + 2.0f*q0*az);
    F[8*EKF_STATE_DIM+1] = dt * ( 2.0f*q3*ax - 2.0f*q0*ay - 2.0f*q1*az);
    F[8*EKF_STATE_DIM+2] = dt * ( 2.0f*q0*ax + 2.0f*q3*ay - 2.0f*q2*az);
    F[8*EKF_STATE_DIM+3] = dt * ( 2.0f*q1*ax + 2.0f*q2*ay + 2.0f*q3*az);
}

// ==================================
// 观测雅可比 H
// ==================================
static void Compute_JacobianH(EKF_Handle_t *ekf)
{
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];
    float32_t *H = ekf->H_data;
    memset(H, 0, EKF_MEAS_DIM * EKF_STATE_DIM * sizeof(float32_t));

    H[0*EKF_STATE_DIM+0] = -2.0f*q2;
    H[0*EKF_STATE_DIM+1] =  2.0f*q3;
    H[0*EKF_STATE_DIM+2] = -2.0f*q0;
    H[0*EKF_STATE_DIM+3] =  2.0f*q1;

    H[1*EKF_STATE_DIM+0] =  2.0f*q1;
    H[1*EKF_STATE_DIM+1] =  2.0f*q0;
    H[1*EKF_STATE_DIM+2] =  2.0f*q3;
    H[1*EKF_STATE_DIM+3] =  2.0f*q2;

    H[2*EKF_STATE_DIM+0] =  2.0f*q0;
    H[2*EKF_STATE_DIM+1] = -2.0f*q1;
    H[2*EKF_STATE_DIM+2] = -2.0f*q2;
    H[2*EKF_STATE_DIM+3] =  2.0f*q3;

    H[3*EKF_STATE_DIM+7] = 1.0f;
}

// ==================================
// 观测模型
// ==================================
static void Observation_Model(const float32_t x[EKF_STATE_DIM], float32_t h[EKF_MEAS_DIM])
{
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    h[0] = 2.0f * (q1*q3 - q0*q2);
    h[1] = 2.0f * (q0*q1 + q2*q3);
    h[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    h[3] = x[EKF_IDX_ALT];
}

// ==================================
// 4×4 矩阵求逆
// ==================================
static arm_status Matrix_Inverse4x4(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst)
{
    float32_t *m = src->pData;
    int n = 4;
    float32_t aug[4][8];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i][j]     = m[i * n + j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < n; col++) {
        int best = col;
        for (int row = col + 1; row < n; row++)
            if (fabsf(aug[row][col]) > fabsf(aug[best][col])) best = row;
        if (best != col)
            for (int j = 0; j < 2 * n; j++) {
                float32_t t = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = t;
            }

        float32_t pivot = aug[col][col];
        if (fabsf(pivot) < 1e-12f) return ARM_MATH_SINGULAR;

        float32_t inv_pivot = 1.0f / pivot;
        for (int j = 0; j < 2 * n; j++)
            aug[col][j] *= inv_pivot;

        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float32_t f = aug[row][col];
            for (int j = 0; j < 2 * n; j++)
                aug[row][j] -= f * aug[col][j];
        }
    }

    float32_t *out = dst->pData;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            out[i * n + j] = aug[i][j + n];

    return ARM_MATH_SUCCESS;
}

// ==================================
// 初始化
// ==================================
void EKF_Init(EKF_Handle_t *ekf)
{
    memset(ekf, 0, sizeof(EKF_Handle_t));

    // 矩阵实例初始化
    arm_mat_init_f32(&ekf->P,      EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->P_data);
    arm_mat_init_f32(&ekf->Q,      EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->Q_data);
    arm_mat_init_f32(&ekf->R,      EKF_MEAS_DIM,   EKF_MEAS_DIM,   ekf->R_data);
    arm_mat_init_f32(&ekf->F,      EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->F_data);
    arm_mat_init_f32(&ekf->H,      EKF_MEAS_DIM,   EKF_STATE_DIM,  ekf->H_data);
    arm_mat_init_f32(&ekf->K,      EKF_STATE_DIM,  EKF_MEAS_DIM,   ekf->K_data);
    arm_mat_init_f32(&ekf->FP,     EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->FP_data);
    arm_mat_init_f32(&ekf->Ft,     EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->Ft_data);
    arm_mat_init_f32(&ekf->S,      EKF_MEAS_DIM,   EKF_MEAS_DIM,   ekf->S_data);
    arm_mat_init_f32(&ekf->S_inv,  EKF_MEAS_DIM,   EKF_MEAS_DIM,   ekf->S_inv_data);
    arm_mat_init_f32(&ekf->HP,     EKF_MEAS_DIM,   EKF_STATE_DIM,  ekf->HP_data);
    arm_mat_init_f32(&ekf->PHt,    EKF_STATE_DIM,  EKF_MEAS_DIM,   ekf->PHt_data);
    arm_mat_init_f32(&ekf->HPHt,   EKF_MEAS_DIM,   EKF_MEAS_DIM,   ekf->HPHt_data);
    arm_mat_init_f32(&ekf->KH,     EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->KH_data);
    arm_mat_init_f32(&ekf->IKH,    EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->IKH_data);
    arm_mat_init_f32(&ekf->Ht,     EKF_STATE_DIM,  EKF_MEAS_DIM,   ekf->Ht_data);
    arm_mat_init_f32(&ekf->IKHt,   EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->IKHt_data);
    arm_mat_init_f32(&ekf->KR,     EKF_STATE_DIM,  EKF_MEAS_DIM,   ekf->KR_data);
    arm_mat_init_f32(&ekf->Kt,     EKF_MEAS_DIM,   EKF_STATE_DIM,  ekf->Kt_data);
    arm_mat_init_f32(&ekf->KRKt,   EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->KRKt_data);
    arm_mat_init_f32(&ekf->P_mid,  EKF_STATE_DIM,  EKF_STATE_DIM,  ekf->P_mid_data);

    // 初始状态: 单位四元数
    ekf->x[0] = 1.0f;

    // 协方差 P
    ekf->P_data[0*EKF_STATE_DIM+0] = 0.01f;   // q0
    ekf->P_data[1*EKF_STATE_DIM+1] = 0.01f;   // q1
    ekf->P_data[2*EKF_STATE_DIM+2] = 0.01f;   // q2
    ekf->P_data[3*EKF_STATE_DIM+3] = 0.01f;   // q3
    ekf->P_data[4*EKF_STATE_DIM+4] = 0.001f;  // bias_x
    ekf->P_data[5*EKF_STATE_DIM+5] = 0.001f;  // bias_y
    ekf->P_data[6*EKF_STATE_DIM+6] = 0.001f;  // bias_z
    ekf->P_data[7*EKF_STATE_DIM+7] = 100.0f;  // altitude
    ekf->P_data[8*EKF_STATE_DIM+8] = 10.0f;   // vel_z

    // 过程噪声 Q
    ekf->Q_data[0*EKF_STATE_DIM+0] = 1e-4f;   // q
    ekf->Q_data[1*EKF_STATE_DIM+1] = 1e-4f;
    ekf->Q_data[2*EKF_STATE_DIM+2] = 1e-4f;
    ekf->Q_data[3*EKF_STATE_DIM+3] = 1e-4f;
    ekf->Q_data[4*EKF_STATE_DIM+4] = 1e-6f;   // bias
    ekf->Q_data[5*EKF_STATE_DIM+5] = 1e-6f;
    ekf->Q_data[6*EKF_STATE_DIM+6] = 1e-6f;
    ekf->Q_data[7*EKF_STATE_DIM+7] = 0.001f;  // altitude
    ekf->Q_data[8*EKF_STATE_DIM+8] = 0.1f;    // vel_z

    // 观测噪声 R
    ekf->R_data[0*EKF_MEAS_DIM+0] = 0.5f;     // accel_x
    ekf->R_data[1*EKF_MEAS_DIM+1] = 0.5f;     // accel_y
    ekf->R_data[2*EKF_MEAS_DIM+2] = 0.5f;     // accel_z
    ekf->R_data[3*EKF_MEAS_DIM+3] = 3.0f;     // baro_altitude

    ekf->alt_initialized = 0;
}


// ==================================
// EKF 更新
// ==================================
void EKF_Update(EKF_Handle_t *ekf, const float32_t accel[3], const float32_t gyro[3],
                float32_t baro_altitude, float32_t dt)
{
    arm_status status;

    if (!ekf->alt_initialized) {
        ekf->x[EKF_IDX_ALT] = baro_altitude;
        ekf->alt_initialized = 1;
    }

    // ===== 保存旧四元数 (用于 Jacobian) =====
    float32_t q_old[4] = {
        ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]
    };

    // ===== 预测 =====
    State_Transition(ekf, gyro, accel, dt);
    Compute_JacobianF(ekf, gyro, accel, dt, q_old);  // 传入旧四元数

    status = arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->FP);
    status = arm_mat_trans_f32(&ekf->F, &ekf->Ft);
    status = arm_mat_mult_f32(&ekf->FP, &ekf->Ft, &ekf->P);
    status = arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P);

    // ===== 更新 =====
    float32_t accel_norm;
    status = arm_sqrt_f32(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2], &accel_norm);
    if (accel_norm < 1e-6f) return;

    float32_t inv_norm = 1.0f / accel_norm;

    ekf->z[0] = accel[0] * inv_norm;
    ekf->z[1] = accel[1] * inv_norm;
    ekf->z[2] = accel[2] * inv_norm;
    ekf->z[3] = baro_altitude;

    Observation_Model(ekf->x, ekf->h);
    arm_sub_f32(ekf->z, ekf->h, ekf->y, EKF_MEAS_DIM);
    Compute_JacobianH(ekf);

    status = arm_mat_mult_f32(&ekf->H, &ekf->P, &ekf->HP);
    status = arm_mat_trans_f32(&ekf->H, &ekf->Ht);
    status = arm_mat_mult_f32(&ekf->HP, &ekf->Ht, &ekf->S);
    status = arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->S);

    status = Matrix_Inverse4x4(&ekf->S, &ekf->S_inv);
    if (status != ARM_MATH_SUCCESS) return;

    status = arm_mat_mult_f32(&ekf->P, &ekf->Ht, &ekf->PHt);
    status = arm_mat_mult_f32(&ekf->PHt, &ekf->S_inv, &ekf->K);

    float32_t Ky[EKF_STATE_DIM];
    arm_matrix_instance_f32 Ky_mat, y_mat;
    arm_mat_init_f32(&Ky_mat, EKF_STATE_DIM, 1, Ky);
    arm_mat_init_f32(&y_mat, EKF_MEAS_DIM, 1, ekf->y);

    status = arm_mat_mult_f32(&ekf->K, &y_mat, &Ky_mat);
    arm_add_f32(ekf->x, Ky, ekf->x, EKF_STATE_DIM);
    Quaternion_Normalize(ekf->x);

    // ===== Joseph 协方差更新 =====
    status = arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->KH);

    for (int i = 0; i < EKF_STATE_DIM; i++)
        for (int j = 0; j < EKF_STATE_DIM; j++)
            ekf->IKH_data[i * EKF_STATE_DIM + j] =
                (i == j) ? 1.0f - ekf->KH_data[i * EKF_STATE_DIM + j]
                         : -ekf->KH_data[i * EKF_STATE_DIM + j];

    float32_t P_old_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P_old;
    arm_mat_init_f32(&P_old, EKF_STATE_DIM, EKF_STATE_DIM, P_old_data);
    memcpy(P_old_data, ekf->P_data, sizeof(P_old_data));

    status = arm_mat_mult_f32(&ekf->IKH, &P_old, &ekf->P_mid);
    status = arm_mat_trans_f32(&ekf->IKH, &ekf->IKHt);
    status = arm_mat_mult_f32(&ekf->P_mid, &ekf->IKHt, &ekf->P);

    status = arm_mat_mult_f32(&ekf->K, &ekf->R, &ekf->KR);
    status = arm_mat_trans_f32(&ekf->K, &ekf->Kt);
    status = arm_mat_mult_f32(&ekf->KR, &ekf->Kt, &ekf->KRKt);
    status = arm_mat_add_f32(&ekf->P, &ekf->KRKt, &ekf->P);
}

// ==================================
// 欧拉角
// ==================================
void EKF_GetEuler(const EKF_Handle_t *ekf, float32_t *roll, float32_t *pitch, float32_t *yaw)
{
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

float32_t EKF_GetAltitude(const EKF_Handle_t *ekf)
{
    return ekf->x[EKF_IDX_ALT];
}

float32_t EKF_GetVelocityZ(const EKF_Handle_t *ekf)
{
    return ekf->x[EKF_IDX_VZ];
}
