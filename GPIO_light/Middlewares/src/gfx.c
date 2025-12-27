#include "gfx.h"
#include "i2coled.h"
#include "font.h"
#include <string.h>
#include <stdlib.h>

// Framebuffer: 128 * 64 bits = 1024 bytes
uint8_t GFX_Buffer[GFX_WIDTH * GFX_HEIGHT / 8];

void GFX_Init(void) {
    // Initialize Hardware via the driver layer
    Init_OLED_Hardware();
    
    // Clear local buffer
    GFX_Clear();
}

void GFX_Clear(void) {
    memset(GFX_Buffer, 0, sizeof(GFX_Buffer));
}

void GFX_Update(void) {
    // Update OLED page by page (8 pages)
    // This logic is required for SH1106 / SSD1306 in Page Mode
    
    for (int i = 0; i < 8; i++) {
        // Calculate pointer to the start of the current page in the buffer
        uint8_t* pPageData = GFX_Buffer + (i * GFX_WIDTH);
        
        // Send this page using the hardware driver
        OLED_UpdatePage_DMA(i, pPageData);
        
        // Wait for DMA to finish this page before starting the next
        // (Because I2C bus is shared and we need to send commands for the next page)
        while (OLED_IsDMABusy());
    }
}

void GFX_DrawPixel(int x, int y, GFX_Color color) {
    if (x < 0 || x >= GFX_WIDTH || y < 0 || y >= GFX_HEIGHT) return;
    
    if (color == GFX_COLOR_WHITE) {
        GFX_Buffer[x + (y / 8) * GFX_WIDTH] |= (1 << (y % 8));
    } else if (color == GFX_COLOR_BLACK) {
        GFX_Buffer[x + (y / 8) * GFX_WIDTH] &= ~(1 << (y % 8));
    } else {
        GFX_Buffer[x + (y / 8) * GFX_WIDTH] ^= (1 << (y % 8));
    }
}

void GFX_DrawLine(int x0, int y0, int x1, int y1, GFX_Color color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    for (;;) {
        GFX_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void GFX_DrawRect(int x, int y, int w, int h, GFX_Color color) {
    GFX_DrawLine(x, y, x + w - 1, y, color);
    GFX_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
    GFX_DrawLine(x + w - 1, y + h - 1, x, y + h - 1, color);
    GFX_DrawLine(x, y + h - 1, x, y, color);
}

void GFX_FillRect(int x, int y, int w, int h, GFX_Color color) {
    for (int i = x; i < x + w; i++) {
        for (int j = y; j < y + h; j++) {
            GFX_DrawPixel(i, j, color);
        }
    }
}

void GFX_DrawChar(int x, int y, char c, GFX_Color color) {
    if (c < ' ' || c > '~') return;
    c -= ' ';
    for (int i = 0; i < 6; i++) {
        uint8_t line = F6x8[c][i];
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                GFX_DrawPixel(x + i, y + j, color);
            } else if (color != GFX_COLOR_INVERT) {
                GFX_DrawPixel(x + i, y + j, GFX_COLOR_BLACK);
            }
        }
    }
}

void GFX_DrawString(int x, int y, const char* str, GFX_Color color) {
    while (*str) {
        GFX_DrawChar(x, y, *str++, color);
        x += 6;
    }
}
