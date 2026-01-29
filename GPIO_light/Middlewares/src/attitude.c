#include "attitude.h"
#include "mpu.h"
#include "kalman.h"


Attitude_Kalman_EKF_t imu_ekf;

void Attitude_Update(float dt, Quaternion* q_bias) {
    // 1. 获取物理单位数据 (以 dps 和 g 为单位)
    float gx = ((int16_t)((mpuDataBuffer[6] << 8) | mpuDataBuffer[7])) / 16.4f;
    float gy = ((int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9])) / 16.4f;
    float gz = ((int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11])) / 16.4f;

    float ax = (int16_t)((mpuDataBuffer[0] << 8) | mpuDataBuffer[1]) / 4096.0f;
    float ay = (int16_t)((mpuDataBuffer[2] << 8) | mpuDataBuffer[3]) / 4096.0f;
    float az = (int16_t)((mpuDataBuffer[4] << 8) | mpuDataBuffer[5]) / 4096.0f;

    float mx = (int16_t)((magDataBuffer[2] << 8) | magDataBuffer[1]);
    float my = (int16_t)((magDataBuffer[4] << 8) | magDataBuffer[3]);
    float mz = (int16_t)((magDataBuffer[6] << 8) | magDataBuffer[5]);

    // 2. 执行四元数 EKF 更新 (单位：dps 需转为 rad/s)
    float deg2rad = 0.01745329f;
    Attitude_Kalman_Update(&imu_ekf, gx * deg2rad, gy * deg2rad, gz * deg2rad, ax, ay, az, mx, my, mz, dt);

    // 3. 应用偏置校正四元数
    imu_ekf.q_corr[0] = imu_ekf.q[0];
    imu_ekf.q_corr[1] = imu_ekf.q[1];
    imu_ekf.q_corr[2] = imu_ekf.q[2];
    imu_ekf.q_corr[3] = imu_ekf.q[3];
    Math3D_QuatMultiply_f32(imu_ekf.q_corr, (float*)q_bias);
}

