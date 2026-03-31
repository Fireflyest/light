#include "command.h"
#include <stdlib.h>
#include <string.h>


static void (*modeCallback)(uint8_t new_mode) = NULL;
static void (*throttleCallback)(float throttle) = NULL;
static void (*heightCallback)(float target_height) = NULL;
static void (*moveCallback)(float distance_x, float distance_y) = NULL;
static void (*attitudeCallback)(float roll, float pitch, float yaw) = NULL;
static void (*armCallback)(void) = NULL;
static void (*emergencyStopCallback)(void) = NULL;
static void (*flightModeCallback)(uint8_t mode) = NULL;

void Command_SetModeCallback(void (*cb)(uint8_t))             { modeCallback = cb; }
void Command_SetThrottleCallback(void (*cb)(float))           { throttleCallback = cb; }
void Command_SetHeightCallback(void (*cb)(float))             { heightCallback = cb; }
void Command_MoveCallback(void (*cb)(float, float))           { moveCallback = cb; }
void Command_SetAttitudeCallback(void (*cb)(float, float, float)) { attitudeCallback = cb; }
void Command_ArmCallback(void (*cb)(void))                    { armCallback = cb; }
void Command_EmergencyStopCallback(void (*cb)(void))          { emergencyStopCallback = cb; }
void Command_FlightModeCallback(void (*cb)(uint8_t))          { flightModeCallback = cb; }


/* ══════════════════════════════════════════════════════════════
 *  小端序 float 读取（安全，不依赖对齐）
 * ══════════════════════════════════════════════════════════════ */
static float ReadFloat(const uint8_t* p)
{
    float f;
    memcpy(&f, p, sizeof(float));
    return f;
}

/* ══════════════════════════════════════════════════════════════
 *  解析并执行
 *
 *  输入格式：[TYPE: 1byte] [DATA: 0~N bytes]
 *
 *  各指令数据布局：
 *    0x10  [mode: u8]                          总长 2
 *    0x11  [throttle: float]                   总长 5
 *    0x12  [height: float]                     总长 5
 *    0x13  [forward: float] [right: float]     总长 9
 *    0x14  [roll: float] [pitch: float]
 *          [yaw: float]                        总长 13
 *    0x15  (无数据)                             总长 1
 *    0x16  (无数据)                             总长 1
 *    0x17  [mode: u8]                          总长 2
 * ══════════════════════════════════════════════════════════════ */
void Command_ParseAndExecute(const uint8_t* data, uint16_t len)
{
    if (len < 1) return;

    uint8_t type = data[0];
    const uint8_t* payload = data + 1;
    uint16_t payloadLen = len - 1;

    switch (type) {

    /* ── 控制模式: [mode: u8] ── */
    case CMD_TYPE_CONTROL_MODE:
        if (payloadLen >= 1 && modeCallback) {
            modeCallback(payload[0]);
        }
        break;

    /* ── 油门: [throttle: float] ── */
    case CMD_TYPE_CONTROL_THROTTLE:
        if (payloadLen >= 4 && throttleCallback) {
            throttleCallback(ReadFloat(payload));
        }
        break;

    /* ── 高度: [height: float] ── */
    case CMD_TYPE_CONTROL_HEIGHT:
        if (payloadLen >= 4 && heightCallback) {
            heightCallback(ReadFloat(payload));
        }
        break;

    /* ── 移动: [forward: float] [right: float] ── */
    case CMD_TYPE_CONTROL_MOVE:
        if (payloadLen >= 8 && moveCallback) {
            moveCallback(ReadFloat(payload), ReadFloat(payload + 4));
        }
        break;

    /* ── 姿态: [roll: float] [pitch: float] [yaw: float] ── */
    case CMD_TYPE_CONTROL_ATTITUDE:
        if (payloadLen >= 12 && attitudeCallback) {
            attitudeCallback(ReadFloat(payload),
                            ReadFloat(payload + 4),
                            ReadFloat(payload + 8));
        }
        break;

    /* ── 解锁: 无数据 ── */
    case CMD_TYPE_CONTROL_ARM:
        if (armCallback) armCallback();
        break;

    /* ── 急停: 无数据 ── */
    case CMD_TYPE_CONTROL_EMERGENCY_STOP:
        if (emergencyStopCallback) emergencyStopCallback();
        break;

    /* ── 飞行模式: [mode: u8] ── */
    case CMD_TYPE_CONTROL_FLIGHT_MODE:
        if (payloadLen >= 1 && flightModeCallback) {
            flightModeCallback(payload[0]);
        }
        break;

    default:
        /* 未知指令，忽略 */
        break;
    }
}