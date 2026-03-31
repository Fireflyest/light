#ifndef __COMMAND_H
#define __COMMAND_H

#include "stm32f4xx.h"

typedef enum {
    /* UI 指令 0x00 ~ 0x0F */
    CMD_TYPE_UI_CLICK              = 0x00,
    CMD_TYPE_UI_UP                 = 0x01,
    CMD_TYPE_UI_DOWN               = 0x02,
    CMD_TYPE_UI_LEFT               = 0x03,
    CMD_TYPE_UI_RIGHT              = 0x04,

    /* 控制指令 0x10 ~ 0x1F */
    CMD_TYPE_CONTROL_MODE          = 0x10,
    CMD_TYPE_CONTROL_THROTTLE      = 0x11,
    CMD_TYPE_CONTROL_HEIGHT        = 0x12,
    CMD_TYPE_CONTROL_MOVE          = 0x13,
    CMD_TYPE_CONTROL_ATTITUDE      = 0x14,
    CMD_TYPE_CONTROL_ARM           = 0x15,
    CMD_TYPE_CONTROL_EMERGENCY_STOP = 0x16,
    CMD_TYPE_CONTROL_FLIGHT_MODE   = 0x17,
} CommandType_t;


void Command_ParseAndExecute(const uint8_t* data, uint16_t len);

void Command_SetModeCallback(void (*callback)(uint8_t new_mode));
void Command_SetThrottleCallback(void (*callback)(float throttle));
void Command_SetHeightCallback(void (*callback)(float target_height));
void Command_MoveCallback(void (*callback)(float distance_x, float distance_y));
void Command_SetAttitudeCallback(void (*callback)(float roll, float pitch, float yaw));
void Command_ArmCallback(void (*callback)(void));
void Command_EmergencyStopCallback(void (*callback)(void));
void Command_FlightModeCallback(void (*callback)(uint8_t mode));



#endif /* __COMMAND_H */