#ifndef __GFX_H
#define __GFX_H

#include "stm32f4xx.h"

// Screen resolution
#define GFX_WIDTH  128
#define GFX_HEIGHT 64

// Colors
typedef enum {
    GFX_COLOR_BLACK = 0,
    GFX_COLOR_WHITE = 1,
    GFX_COLOR_INVERT = 2
} GFX_Color;

// FrameBuffer
extern uint8_t GFX_Buffer[GFX_WIDTH * GFX_HEIGHT / 8];

// Initialization
void GFX_Init(void);

// Core drawing functions
void GFX_Clear(void);
void GFX_Update(void);
void GFX_DrawPixel(int x, int y, GFX_Color color);
void GFX_DrawLine(int x0, int y0, int x1, int y1, GFX_Color color);
void GFX_DrawRect(int x, int y, int w, int h, GFX_Color color);
void GFX_FillRect(int x, int y, int w, int h, GFX_Color color);
void GFX_DrawCircle(int x0, int y0, int r, GFX_Color color);
void GFX_DrawChar(int x, int y, char c, GFX_Color color);
void GFX_DrawString(int x, int y, const char* str, GFX_Color color);

#endif /* __GFX_H */
