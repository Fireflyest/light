#include "attitude.h"
#include "icm20948.h"
#include "spatial_math.h"


// Estimator_Attitude_EKF_t imu_ekf;

uint8_t imu_rx_buf[14];
uint8_t mag_rx_buf[6];
float altitude_rx;
float temperature_rx;

sm_quat_t last_quat;
LowPass_Filter_t is_still;
uint8_t accel_face = 0;

EKF_Handle_t imu_ekf;

static Calib_Handle_t calib_handle;
static Accel_Calib_t accel_calib;

static const float deg2rad = 0.01745329f;
static const float g = 9.80665f;


void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale) {
    // sm_vec3_t mag_ref = {1.0f, 0.0f, 0.0f}; // 需标定或配置，单位向量
    // Estimator_Attitude_Init(&imu_ekf, mag_ref);

    EKF_Init(&imu_ekf);
    LowPass_Filter_Init(&is_still, 0.1f, 0); // 初始化低通滤波器，alpha=0.1，初始输出为0

    if (accel_bias[0] != 0 && accel_scale[0] != 1.0f) {
        accel_calib.is_valid = 1;
        for (int i = 0; i < 3; i++) {
            accel_calib.bias[i] = accel_bias[i];
            accel_calib.scale[i] = accel_scale[i];
        }
    } else {
        Calibrate_Init(&calib_handle);
        Calibrate_Start(&calib_handle);
    }
}

void Attitude_Update(float dt) {
    // 1. 获取物理单位数据 (以 dps 和 g 为单位)
    float gx = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f * deg2rad;
    float gy = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f * deg2rad;
    float gz = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f * deg2rad;

    float ax = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 2048.0f * g;
    float ay = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 2048.0f * g;
    float az = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 2048.0f * g;

    float mx = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) * 0.15f; // 0.15 μT/LSB
    float my = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) * 0.15f; // 0.15 μT/LSB
    float mz = (int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) * 0.15f; // 0.15 μT/LSB

    sm_vec3_t gyro = {gx, gy, gz};
    sm_vec3_t accel = {ax, ay, az};
    sm_vec3_t mag = {mx, my, mz};

    if (calib_handle.state == CALIB_COLLECTING && is_still.output < 0.04f * deg2rad) {
        Calibrate_AddSample(&calib_handle, accel, accel_face);
        if (calib_handle.state == CALIB_DONE) {
            accel_calib = calib_handle.calib;
            Persistence_WriteCalibData(-1, accel_calib.bias, accel_calib.scale);
        }
    }

    if (accel_calib.is_valid) {
        float calibrated_accel[3];
        Calibrate_Apply(&accel_calib, accel, calibrated_accel);
        accel[0] = calibrated_accel[0];
        accel[1] = calibrated_accel[1];
        accel[2] = calibrated_accel[2];
    }
    
    EKF_Update(&imu_ekf, accel, gyro, altitude_rx, dt);
}

void Attitude_IsStill(uint8_t *still) {
    sm_quat_t current_quat;
    memcpy(current_quat, imu_ekf.x, sizeof(sm_quat_t));

    float diff_angle = Spatial_QuatAngleBetween(current_quat, last_quat);
    LowPass_Update(&is_still, diff_angle);
    *still = is_still.output < 0.04f * deg2rad;

    if (is_still.output > 3 * deg2rad && Calibrate_IsFaceDone(&calib_handle, accel_face)) {
        accel_face++;
    }

    memcpy(last_quat, current_quat, sizeof(sm_quat_t));
}

