#ifndef __OLED_H__
#define __OLED_H__

#include "delay.h"
#include "stm32f4xx_conf.h"

// Font and bitmap definitions directly in the header
#ifdef OLED_DEFINE_FONTS

#include "bmp.h"
#include "oledfont.h"

#else
// External declarations
extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];
extern const unsigned char BMP1[];
extern const unsigned char BMP2[];
extern const unsigned char BMP3[];
extern const unsigned char BMP4[];
extern const unsigned char BMP5[];
extern const unsigned char BMP6[];
extern const unsigned char Hzk[][32];
#endif

#define OLED          0
#define FONT_SIZE     8        // 8 or 16
#define XLevelL       0x00
#define XLevelH       0x10
#define Max_Column    128
#define Max_Row       64
#define Brightness    0xff
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
#define OLED_I2C I2C1
// OLED I2C address
#define OLED_I2C_ADDRESS 0x78

// OLED控制用函数
extern void OLED_WR_Byte(uint8_t dat, OLED_WR_MODE cmd);
extern void OLED_Display_Status(DIS_MODE mode);
extern void OLEDConfiguration(void);
extern void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
extern void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
extern void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
extern void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
extern void OLED_ShowString(uint8_t x, uint8_t y, uint8_t* p);
extern void OLED_Set_Pos(uint8_t x, uint8_t y);
extern void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
extern void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]);

#endif
