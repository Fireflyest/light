#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "stm32f4xx.h"

#define CALIB_SAMPLES_PER_FACE 300


// 加速度计校准参数（6面校准）
typedef struct {
    float bias[3];      // 零偏 (m/s²)
    float scale[3];     // 比例因子 (无量纲，理想为1)
    uint8_t is_valid;   // 校准是否有效
} Accel_Calib_t;

// 校准状态
typedef enum {
    CALIB_IDLE = 0,
    CALIB_COLLECTING,
    CALIB_DONE,
    CALIB_FAILED
} Calib_State_t;

// 校准句柄
typedef struct {
    Accel_Calib_t calib;
    Calib_State_t state;

    float face_min[6][3];  // [face][axis]
    float face_max[6][3];  // [face][axis]
    float face_done[6];
    uint32_t face_count[6];
    uint8_t current_face;  // 0-5
    
    float samples[CALIB_SAMPLES_PER_FACE * 6][3];
    uint32_t sample_count;
} Calib_Handle_t;

// 接口函数
void Calibrate_Init(Calib_Handle_t *handle);
void Calibrate_Start(Calib_Handle_t *handle);
void Calibrate_AddSample(Calib_Handle_t *handle, const float accel[3], uint8_t face);
uint8_t Calibrate_IsFaceDone(Calib_Handle_t *handle, uint8_t face);
void Calibrate_Compute(Calib_Handle_t *handle);
void Calibrate_Apply(const Accel_Calib_t *calib, const float raw[3], float calibrated[3]);
void Calibrate_Load(Calib_Handle_t *handle, const Accel_Calib_t *calib);

#endif /* __CALIBRATE_H */