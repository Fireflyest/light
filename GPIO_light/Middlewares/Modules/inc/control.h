#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "pid.h"
#include "lowpass.h"
#include "spatial_math.h"

extern PID_t pidRoll, pidPitch, pidYaw, pidHeight;
extern PID_t pidRateRoll, pidRatePitch, pidRateYaw;
extern float baseThrottle;
extern __IO float rateSetRoll, rateSetPitch, rateSetYaw;
extern __IO float thrustOutput;


#define RATE_LOOP_HZ     1000
#define ATTITUDE_LOOP_HZ 200
#define GYRO_TAU         0.004f
#define DEG2RAD          0.017453292519943296f
#define RAD2DEG          57.29577951308232f


typedef enum {
    CONTROL_MODE_MANUAL   = 0,
    CONTROL_MODE_ATTITUDE = 1,
    CONTROL_MODE_VELOCITY = 2,
    CONTROL_MODE_POSITION = 3,
} ControlMode_t;

typedef enum {
    FLIGHT_MODE_TAKEOFF   = 0,
    FLIGHT_MODE_HOVER     = 1,
    FLIGHT_MODE_LAND      = 2,
    FLIGHT_MODE_MANUAL    = 3,
    FLIGHT_MODE_EMERGENCY = 4,
    FLIGHT_MODE_NUM,
} FlightMode_t;


void Control_Init(uint32_t freq);
void ControlAttitude_Loop(void);
void ControlMotor_Loop(void);

void Control_SetMode(uint8_t new_mode);
void Control_SetThrottle(float throttle);
void Control_SetHeight(float height);
void Control_Move(float forward, float right);
void Control_SetAttitude(float roll, float pitch, float yaw);
void Control_Arm(void);
void Control_EmergencyStop(void);
void Control_FlightMode(uint8_t mode);

#endif /* __CONTROL_H */