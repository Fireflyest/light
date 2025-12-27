#ifndef __OLED_H
#define __OLED_H

#include "usystem.h"
#include "stm32f4xx_conf.h"
#include "bmp.h"
#include "font.h"



#define OLED          0
#define FONT_SIZE     8        // 8 or 16
#define XLevelL       0x00
#define XLevelH       0x10
#define MAX_COLUMNS   128
#define MAX_ROWS      8
#define X_WIDTH       128
#define Y_WIDTH       64

typedef enum {
    OLED_CMD,
    OLED_DATA
} OLED_WR_MODE;

typedef enum {
    Display_ON,
    Display_OFF,
    Display_Clear,
    Display_Test
} DIS_MODE;

// I2C peripheral to use
#define TARGET_I2C I2C1
// OLED I2C address
#define TARGET__I2C_ADDRESS 0x78

// OLED控制用函数
extern void OLEDConfiguration(void);
extern void OLED_WR_Byte(uint8_t dat, OLED_WR_MODE cmd);
extern void OLED_Display_Status(DIS_MODE mode);
extern void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
extern void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
extern void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
extern void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
extern void OLED_ShowString(uint8_t x, uint8_t y, uint8_t* p);
extern void OLED_Set_Pos(uint8_t x, uint8_t y);
extern void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
extern void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]);

#endif // __OLED_H
