#include "locate.h"
#include "mpu.h"


LowPass_Filter_t tempFilt, pressFilt;
Locate_Kalman_EKF_t loc_ekf;

float temperature;
float barometricPressure;
float altitude;

static int32_t t_fine;
static uint8_t lowPassInited = 0;

static void BMP_Compensate_T(int32_t adc_T);
static void BMP_Compensate_P(int32_t adc_P);
static void BMP_GetAltitude();


void Locate_Update(float dt, float32_t q[4]) {
    uint32_t adc_P = ((uint32_t)bmpDataBuffer[0] << 12) | ((uint32_t)bmpDataBuffer[1] << 4) | ((bmpDataBuffer[2] >> 4) & 0x0F);
    uint32_t adc_T = ((uint32_t)bmpDataBuffer[3] << 12) | ((uint32_t)bmpDataBuffer[4] << 4) | ((bmpDataBuffer[5] >> 4) & 0x0F);
    BMP_Compensate_T(adc_T);
    BMP_Compensate_P(adc_P);

    if (!lowPassInited) {
        LowPass_Filter_Init(&tempFilt, 1.0f, temperature);
        LowPass_Filter_Init(&pressFilt, 1.0f, barometricPressure);
        lowPassInited = 1;
    }

    LowPass_UpdateWithTau(&tempFilt, temperature, TEMP_TAU, dt);
    temperature = tempFilt.output;
    LowPass_UpdateWithTau(&pressFilt, barometricPressure, PRESS_TAU, dt);
    barometricPressure = pressFilt.output;

    BMP_GetAltitude();

    // 获取加速度
    float az = (int16_t)((mpuDataBuffer[4] << 8) | mpuDataBuffer[5]) / 4096.0f;
    float ay = (int16_t)((mpuDataBuffer[2] << 8) | mpuDataBuffer[3]) / 4096.0f;
    float ax = (int16_t)((mpuDataBuffer[0] << 8) | mpuDataBuffer[1]) / 4096.0f;

    float az_world = (1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * az
                      + (2.0f * (q[0]*q[2] - q[3]*q[1])) * ay
                      + (2.0f * (q[0]*q[1] + q[3]*q[2])) * ax;
    
    float az_linear = az_world + 9.81f; // 去重力加速度

    // 执行高度 EKF 更新
    Locate_Kalman_Update(&loc_ekf, az_linear, altitude, dt);
}


void BMP_Compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
}

void BMP_Compensate_P(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)bmp_calib.dig_P1) >> 33;
    if (var1 == 0) return; // avoid div0
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    barometricPressure = p / 25600.0f + BAROMETERIC_PRESSURE_OFFSET; // convert fixed-point to float Pa
}

void BMP_GetAltitude() {
    float T = temperature + 273.15f; 
    // 0.190263 是 R*L/g 的常数
    const float EXP = 0.190263f; 
    
    // 使用实时温度 T 替代固定常数 44330 (44330 实际上包含了 288.15K 的假设)
    // 高度 = (T / 0.0065) * (1 - (P/P0)^EXP)
    altitude = (T / 0.0065f) * (1.0f - powf(barometricPressure / SEA_LEVEL_PRESSURE_HPA, EXP));
}