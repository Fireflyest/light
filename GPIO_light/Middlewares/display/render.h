#ifndef __RENDER_H
#define __RENDER_H

#include "labels.h"
#include "oled.h"

#define M_PI 3.14159265358979323846

typedef enum {
    RENDER_IDLE,    // 空闲
    RENDER_DRAWING, // 绘制中
    RENDER_ERROR    // 错误
} RenderState;

typedef enum {
    DRAW_ON,        // 亮
    DRAW_OFF,       // 灭
    DRAW_SWITCH     // 切换
} DrawType;

typedef enum {
    LINE_NONE,          // 无线条
    LINE_SOLID,         // 实线
    LINE_DASHED,        // 虚线
} LineType;

typedef enum {
    FILL_NONE,     // 无填充
    FILL_SOLID,    // 实心填充
    FILL_PATTERN,  // 图案填充
} FillType;

typedef enum {
    QUAD_TOP_LEFT,     // 左上
    QUAD_BOTTOM_LEFT,  // 左下
    QUAD_TOP_RIGHT,    // 右上
    QUAD_BOTTOM_RIGHT  // 右下
} QuadrantType;

void Render_Init(void);
void Render_Test(void);
void Render_FrameBuffer(void);

void Draw_Byte(uint8_t x, uint8_t y, uint8_t dat, DrawType type);
void Draw_Pixel(uint8_t x, uint8_t y, DrawType type);
void Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LineType line, DrawType type);
void Draw_Rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LineType line, FillType fillType, DrawType type);
void Draw_RoundedRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t r, LineType line, FillType fillType, DrawType type);
void Draw_Arc(uint8_t x, uint8_t y, uint8_t r, QuadrantType quadrant, LineType line, DrawType type);
void Draw_Quadrant(uint8_t x, uint8_t y, uint8_t r, QuadrantType quadrant, LineType line, FillType fill, DrawType type);


#endif /* __RENDER_H */