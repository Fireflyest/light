#include "ekf_state7.h"

#include <string.h>
#include <math.h>
#include "arm_math.h"

// ==================================
// 内部辅助函数
// ==================================
static void Quaternion_Multiply(const float32_t q1[4], const float32_t q2[4], float32_t q_out[4]) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void Quaternion_Normalize(float32_t q[4]) {
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

static void State_Transition(EKF_Handle_t *ekf, const float32_t gyro[3], float32_t dt) {
    float32_t *x = ekf->x;

    float32_t omega[3] = {
        gyro[0] - x[4],
        gyro[1] - x[5],
        gyro[2] - x[6]
    };

    float32_t omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float32_t q_dot[4];

    Quaternion_Multiply(x, omega_q, q_dot);      // q_dot = q * omega_q
    arm_scale_f32(q_dot, 0.5f * dt, q_dot, 4);   // q_dot *= 0.5*dt
    arm_add_f32(x, q_dot, x, 4);                  // x += q_dot
    Quaternion_Normalize(x);
}

static void Compute_JacobianF(EKF_Handle_t *ekf, const float32_t gyro[3], float32_t dt) {
    float32_t *x = ekf->x;
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float32_t wx = gyro[0] - x[4];
    float32_t wy = gyro[1] - x[5];
    float32_t wz = gyro[2] - x[6];
    float32_t *F = ekf->F_data;

    memset(F, 0, EKF_STATE_DIM * EKF_STATE_DIM * sizeof(float32_t));
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        F[i * EKF_STATE_DIM + i] = 1.0f;
    }

    float32_t half_dt = 0.5f * dt;
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
}

static void Compute_JacobianH(EKF_Handle_t *ekf) {
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

    // ∂z_alt/∂h = 1
    H[3*EKF_STATE_DIM+7] = 1.0f;
}

static void Observation_Model(const float32_t x[EKF_STATE_DIM], float32_t h[EKF_MEAS_DIM]) {
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    h[0] = 2.0f * (q1*q3 - q0*q2);
    h[1] = 2.0f * (q0*q1 + q2*q3);
    h[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    h[3] = x[EKF_IDX_ALT];
}

static arm_status Matrix_Inverse4x4(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst) {
    float32_t *m = src->pData;
    int n = 4;
    float32_t aug[4][8];

    // 构建增广矩阵 [A | I]
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i][j]     = m[i * n + j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 高斯-约旦消元
    for (int col = 0; col < n; col++) {
        // 选主元
        int best = col;
        for (int row = col + 1; row < n; row++)
            if (fabsf(aug[row][col]) > fabsf(aug[best][col])) best = row;
        if (best != col)
            for (int j = 0; j < 2 * n; j++) {
                float32_t t = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = t;
            }

        float32_t pivot = aug[col][col];
        if (fabsf(pivot) < 1e-12f) return ARM_MATH_SINGULAR;

        // 归一化当前行
        float32_t inv_pivot = 1.0f / pivot;
        for (int j = 0; j < 2 * n; j++)
            aug[col][j] *= inv_pivot;

        // 消去其他行
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float32_t f = aug[row][col];
            for (int j = 0; j < 2 * n; j++)
                aug[row][j] -= f * aug[col][j];
        }
    }

    // 提取逆矩阵
    float32_t *out = dst->pData;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            out[i * n + j] = aug[i][j + n];

    return ARM_MATH_SUCCESS;
}

// ==================================
// 公开接口
// ==================================
void EKF_Init(EKF_Handle_t *ekf) {
    memset(ekf, 0, sizeof(EKF_Handle_t));
    ekf->x[0] = 1.0f;

    arm_mat_init_f32(&ekf->P, EKF_STATE_DIM, EKF_STATE_DIM, ekf->P_data);
    arm_mat_init_f32(&ekf->Q, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Q_data);
    arm_mat_init_f32(&ekf->R, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->R_data);
    arm_mat_init_f32(&ekf->F, EKF_STATE_DIM, EKF_STATE_DIM, ekf->F_data);
    arm_mat_init_f32(&ekf->H, EKF_MEAS_DIM, EKF_STATE_DIM, ekf->H_data);
    arm_mat_init_f32(&ekf->K, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->K_data);
    arm_mat_init_f32(&ekf->FP, EKF_STATE_DIM, EKF_STATE_DIM, ekf->FP_data);
    arm_mat_init_f32(&ekf->Ft, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Ft_data);
    arm_mat_init_f32(&ekf->S, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->S_data);
    arm_mat_init_f32(&ekf->S_inv, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->S_inv_data);
    arm_mat_init_f32(&ekf->HP, EKF_MEAS_DIM, EKF_STATE_DIM, ekf->HP_data);
    arm_mat_init_f32(&ekf->PHt, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->PHt_data);
    arm_mat_init_f32(&ekf->HPHt, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->HPHt_data);
    arm_mat_init_f32(&ekf->KH, EKF_STATE_DIM, EKF_STATE_DIM, ekf->KH_data);
    arm_mat_init_f32(&ekf->IKH, EKF_STATE_DIM, EKF_STATE_DIM, ekf->IKH_data);

    // 四元数
    for (int i = 0; i < 4; i++) {
        ekf->P_data[i * EKF_STATE_DIM + i] = 0.01f;
        ekf->Q_data[i * EKF_STATE_DIM + i] = 1e-4f;
    }
    // 陀螺仪偏置
    for (int i = 4; i < 7; i++) {
        ekf->P_data[i * EKF_STATE_DIM + i] = 0.001f;
        ekf->Q_data[i * EKF_STATE_DIM + i] = 1e-6f;
    }
    // 高度
    ekf->P_data[7 * EKF_STATE_DIM + 7] = 10.0f;    // 初始不确定度大
    ekf->Q_data[7 * EKF_STATE_DIM + 7] = 0.001f;    // 随机游走噪声

    // 观测噪声
    for (int i = 0; i < 3; i++) {
        ekf->R_data[i * EKF_MEAS_DIM + i] = 0.5f;   // ★ 改3→EKF_MEAS_DIM
    }
    // 高度观测噪声
    ekf->R_data[3 * EKF_MEAS_DIM + 3] = 3.0f;       // 气压高度噪声, 根据实际调
}

void EKF_Update(EKF_Handle_t *ekf, const float32_t accel[3], const float32_t gyro[3],
                float32_t baro_altitude, float32_t dt) {
    arm_status status;

    // 预测
    State_Transition(ekf, gyro, dt);
    Compute_JacobianF(ekf, gyro, dt);

    status = arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->FP);
    status = arm_mat_trans_f32(&ekf->F, &ekf->Ft);
    status = arm_mat_mult_f32(&ekf->FP, &ekf->Ft, &ekf->P);
    status = arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P);

    // 更新
    float32_t accel_norm;
    status = arm_sqrt_f32(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2], &accel_norm);
    if (accel_norm < 1e-6f) return;

    float32_t inv_norm = 1.0f / accel_norm;
    ekf->z[0] = accel[0] * inv_norm;
    ekf->z[1] = accel[1] * inv_norm;
    ekf->z[2] = accel[2] * inv_norm;
    // 气压高度观测
    ekf->z[3] = baro_altitude;

    Observation_Model(ekf->x, ekf->h);

    // 3 → EKF_MEAS_DIM
    arm_sub_f32(ekf->z, ekf->h, ekf->y, EKF_MEAS_DIM);

    Compute_JacobianH(ekf);

    status = arm_mat_mult_f32(&ekf->H, &ekf->P, &ekf->HP);

    // Ht 维度: (EKF_MEAS_DIM × EKF_STATE_DIM)^T = EKF_STATE_DIM × EKF_MEAS_DIM
    static arm_matrix_instance_f32 Ht;
    static float32_t Ht_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_mat_init_f32(&Ht, EKF_STATE_DIM, EKF_MEAS_DIM, Ht_data);

    status = arm_mat_trans_f32(&ekf->H, &Ht);
    status = arm_mat_mult_f32(&ekf->HP, &Ht, &ekf->S);
    status = arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->S);

    // 4×4
    status = Matrix_Inverse4x4(&ekf->S, &ekf->S_inv);
    if (status != ARM_MATH_SUCCESS) return;

    status = arm_mat_mult_f32(&ekf->P, &Ht, &ekf->PHt);
    status = arm_mat_mult_f32(&ekf->PHt, &ekf->S_inv, &ekf->K);

    float32_t Ky[EKF_STATE_DIM];
    arm_matrix_instance_f32 Ky_mat, y_mat;
    arm_mat_init_f32(&Ky_mat, EKF_STATE_DIM, 1, Ky);
    arm_mat_init_f32(&y_mat, EKF_MEAS_DIM, 1, ekf->y);

    status = arm_mat_mult_f32(&ekf->K, &y_mat, &Ky_mat);
    arm_add_f32(ekf->x, Ky, ekf->x, EKF_STATE_DIM);
    Quaternion_Normalize(ekf->x);

    status = arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->KH);

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            if (i == j) {
                ekf->IKH_data[i * EKF_STATE_DIM + j] = 1.0f - ekf->KH_data[i * EKF_STATE_DIM + j];
            } else {
                ekf->IKH_data[i * EKF_STATE_DIM + j] = -ekf->KH_data[i * EKF_STATE_DIM + j];
            }
        }
    }

    float32_t P_temp_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P_temp;
    arm_mat_init_f32(&P_temp, EKF_STATE_DIM, EKF_STATE_DIM, P_temp_data);
    memcpy(P_temp_data, ekf->P_data, sizeof(P_temp_data));
    status = arm_mat_mult_f32(&ekf->IKH, &P_temp, &ekf->P);
}

void EKF_GetEuler(const EKF_Handle_t *ekf, float32_t *roll, float32_t *pitch, float32_t *yaw) {
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

float32_t EKF_GetAltitude(const EKF_Handle_t *ekf)
{
    return ekf->x[EKF_IDX_ALT];
}
