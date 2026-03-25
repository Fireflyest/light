#include "attitude.h"
#include "icm20948.h"
#include "spatial_math.h"


uint8_t imu_rx_buf[14];
uint8_t mag_rx_buf[6];
float altitude_rx;
float temperature_rx;

static sm_vec3_t gyro_current = {0};
static sm_vec3_t accel_current = {0};
static sm_vec3_t mag_current = {0};

static uint8_t accel_clib_face = 0;

static sm_quat_t last_quat;
static LowPass_Filter_t diff_angle_filter;
static LowPass_Filter_t altitude_filter;

static EKF_Handle_t imu_ekf;

static Calib_Handle_t calib_handle;
static Accel_Calib_t accel_calib;

static const float deg2rad = 0.01745329f;
static const float g = 9.80665f;
static const float still_threshold = 0.046f * deg2rad;

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale) {
    EKF_Init(&imu_ekf);
    LowPass_Filter_Init(&diff_angle_filter, 0.1f, 0); // 初始化低通滤波器，alpha=0.1，初始输出为0
    LowPass_Filter_Init(&altitude_filter, 0.1f, 0); // 初始化高度低通滤波器，alpha=0.1，初始输出为0

    if (accel_bias[0] != 0 && accel_scale[0] != 1.0f) {
        accel_calib.is_valid = 1;
        for (int i = 0; i < 3; i++) {
            accel_calib.bias[i] = accel_bias[i];
            accel_calib.scale[i] = accel_scale[i];
        }
    } else {
        Attitude_Calibrate();
    }
}

void Attitude_Update(float dt) {
    // 1. 获取物理单位数据 (以 dps 和 g 为单位)
    gyro_current[0] = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f * deg2rad;
    gyro_current[1] = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f * deg2rad;
    gyro_current[2] = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f * deg2rad;

    accel_current[0] = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 2048.0f * g;
    accel_current[1] = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 2048.0f * g;
    accel_current[2] = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 2048.0f * g;

    mag_current[0] = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) * 0.15f; // 0.15 μT/LSB
    mag_current[1] = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) * 0.15f; // 0.15 μT/LSB
    mag_current[2] = (int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) * 0.15f; // 0.15 μT/LSB

    if (calib_handle.state == CALIB_COLLECTING && diff_angle_filter.output < still_threshold) {
        Calibrate_AddSample(&calib_handle, accel_current, accel_clib_face);
        if (calib_handle.state == CALIB_DONE) {
            accel_calib = calib_handle.calib;
            Persistence_WriteCalibData(-1, accel_calib.bias, accel_calib.scale);
        }
    }

    if (accel_calib.is_valid) {
        float calibrated_accel[3];
        Calibrate_Apply(&accel_calib, accel_current, calibrated_accel);
        accel_current[0] = calibrated_accel[0];
        accel_current[1] = calibrated_accel[1];
        accel_current[2] = calibrated_accel[2];
    }

    LowPass_Update(&altitude_filter, altitude_rx);
    
    EKF_Update(&imu_ekf, accel_current, gyro_current, altitude_filter.output, dt);
}

void Attitude_IsStill(uint8_t *still) {
    sm_quat_t current_quat;
    memcpy(current_quat, imu_ekf.x, sizeof(sm_quat_t));

    float diff_angle = Spatial_QuatAngleBetween(current_quat, last_quat);
    LowPass_Update(&diff_angle_filter, diff_angle);
    *still = diff_angle_filter.output < still_threshold;

    if (diff_angle_filter.output > 3 * deg2rad && Calibrate_IsFaceDone(&calib_handle, accel_clib_face)) {
        accel_clib_face++;
    }

    memcpy(last_quat, current_quat, sizeof(sm_quat_t));
}

void Attitude_GetEuler(float *yaw, float *pitch, float *roll) {
    Spatial_QuatGetEuler(yaw, pitch, roll, imu_ekf.x);
}

void Attitude_GetQuat(sm_quat_t q) {
    memcpy(q, imu_ekf.x, sizeof(sm_quat_t));
}

void Attitude_GetGyro(sm_vec3_t gyro) {
    memcpy(gyro, gyro_current, sizeof(sm_vec3_t));
    gyro[0] -= imu_ekf.x[4];
    gyro[1] -= imu_ekf.x[5];
    gyro[2] -= imu_ekf.x[6];
}

void Attitude_GetAccel(sm_vec3_t accel) {
    memcpy(accel, accel_current, sizeof(sm_vec3_t));
}

// void Attitude_GetMag(sm_vec3_t mag);

void Attitude_GetAltitude(float *altitude) {
    *altitude = EKF_GetAltitude(&imu_ekf);
}



void Attitude_Calibrate(void) {
    Calibrate_Init(&calib_handle);
    Calibrate_Start(&calib_handle);
}

void Attitude_CalibratingFace(uint8_t *face) {
    *face = accel_clib_face;
}