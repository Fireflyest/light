#include "attitude.h"
#include "icm20948.h"


Estimator_Attitude_EKF_t imu_ekf;

void Attitude_Init(void) {
    sm_vec3_t mag_ref = {1.0f, 0.0f, 0.0f}; // 需标定或配置，单位向量
    Estimator_Attitude_Init(&imu_ekf, mag_ref);
}

void Attitude_Update(float dt) {
    // 1. 获取物理单位数据 (以 dps 和 g 为单位)
    float gx = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f;
    float gy = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f;
    float gz = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f;

    float ax = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 4096.0f;
    float ay = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 4096.0f;
    float az = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 4096.0f;

    float mx = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) * 0.15f; // 0.15 μT/LSB
    float my = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) * 0.15f; // 0.15 μT/LSB
    float mz = (int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) * 0.15f; // 0.15 μT/LSB

    static const float deg2rad = 0.01745329f;
    static const float g = 9.80665f;
    sm_vec3_t gyro = {gx * deg2rad, gy * deg2rad, gz * deg2rad};
    sm_vec3_t accel = {ax * g, ay * g, az * g};
    sm_vec3_t mag = {mx, my, mz};

    // 2. 执行四元数 EKF 更新 (单位：dps 需转为 rad/s)
    Estimator_Attitude_Update(&imu_ekf, gyro, accel, mag, dt);
}

