# include "usystem.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

__IO uint16_t sysTick;

# ifdef DISPLAY_ENABLE
UI_Logger logWindow;
uint16_t screenFps = 0;
uint16_t logicFps = 0;
UI_Widget* widgets[16];
static void UI_MPU_BMP_Draw(UI_Widget* widget);
static void UI_Cube_Draw(UI_Widget* widget);
# endif

typedef enum { 
    STATE_NONE,
    STATE_HOME,
    STATE_MPU,
    STATE_CUBE
} AppState;
AppState currentState;

void delay_ms(__IO uint32_t nTime) {
    uint32_t start = sysTick;
    while((sysTick - start) < nTime);
}


void Init_USystem() {
    Init_Key();
    Init_LED();
}

void Init_Display() {
    # ifdef DISPLAY_ENABLE
    Init_OLED_Hardware();
    Init_GFX();
    Init_UI();
    # endif
}

void Init_USART(uint16_t baudrate) {
    Init_DMA_For_USART1_RX(rxBufferUart1, sizeof(rxBufferUart1));
    Init_DMA_For_USART1_TX(txBufferUart1);
    Init_USART1(baudrate);
}

void Init_PWM(uint16_t period, uint16_t prescaler) {
    Init_PWM_TIM(period, prescaler);
    Init_DMA_For_PWM_TIM3(pwmDutyBuffer);
}

void Init_MPU() {
    Init_IMU_Hardware();
}

void Init_Widgets() {
    # ifdef DISPLAY_ENABLE
    widgets[STATE_NONE] = (UI_Widget*)&logWindow;
    currentState = STATE_NONE;

    static UI_Window mpuWindow;
    UI_Window_Init(&mpuWindow, 0, 0, 128, 64);
    mpuWindow.base.draw = UI_MPU_BMP_Draw;
    widgets[STATE_MPU] = (UI_Widget*)&mpuWindow;

    static UI_Window homeWindow;
    UI_Window_Init(&homeWindow, 0, 0, 128, 64);
    static UI_Label lblTitle, lblCount;
    UI_Label_Init(&lblTitle, 10, 10, "Home:");
    UI_Label_Init(&lblCount, 50, 30, "aaaaaa");
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblTitle);
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblCount);
    widgets[STATE_HOME] = (UI_Widget*)&homeWindow;

    static UI_Window cubeWindow;
    UI_Window_Init(&cubeWindow, 0, 0, 128, 64);
    cubeWindow.base.draw = UI_Cube_Draw;
    widgets[STATE_CUBE] = (UI_Widget*)&cubeWindow;
    # endif
}

uint16_t Read_Bluetooth_Command(uint8_t* buffer) {
    uint16_t len = 0;
    if (rxStatusUart1) {
        len = Read_USART1_Data(buffer);
    }
    return len;
}



# ifdef DISPLAY_ENABLE
float angleX = 0, angleY = 0, angleZ = 0;
void UI_MPU_BMP_Draw(UI_Widget* widget) {
    char line[40];
    int x = widget->x + 2;
    int y = widget->y + 2;
    int lh = 10; // 行高，根据字体调整

    int16_t ax = (int16_t)((mpuDataBuffer[0] << 8) | mpuDataBuffer[1]);
    int16_t ay = (int16_t)((mpuDataBuffer[2] << 8) | mpuDataBuffer[3]);
    int16_t az = (int16_t)((mpuDataBuffer[4] << 8) | mpuDataBuffer[5]);

    int16_t gx = (int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9]);
    int16_t gy = (int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11]);
    int16_t gz = (int16_t)((mpuDataBuffer[12] << 8) | mpuDataBuffer[13]);

    int16_t tempRaw = (int16_t)((mpuDataBuffer[6] << 8) | mpuDataBuffer[7]);
    float tempC = (float)tempRaw / 333.87f + 21.0f;

    int16_t p1 = (int16_t)((bmpDataBuffer[0] << 12) | (bmpDataBuffer[1] << 4) | (bmpDataBuffer[2] >> 4));
    int16_t p2 = (int16_t)((bmpDataBuffer[3] << 12) | (bmpDataBuffer[4] << 4) | (bmpDataBuffer[5] >> 4));

    snprintf(line, sizeof(line), "AX %6d GX %6d", (int)ax, (int)gx);
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AY %6d GY %6d", (int)ay, (int)gy);
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AZ %6d GZ %6d", (int)az, (int)gz);
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);
    
    {
        float at = tempC < 0.0f ? -tempC : tempC;
        int ti = (int)at;                       // integer part
        int tf = (int)(at * 10.0f) % 10;         // one decimal
        if (tempC < 0.0f) {
            snprintf(line, sizeof(line), "T -%d.%dC", ti, tf);
        } else {
            snprintf(line, sizeof(line), "T %d.%dC", ti, tf);
        }
    }
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);


    {
        float at = temperature < 0.0f ? -temperature : temperature;
        int t_i = (int)at;
        int t_d = (int)(at * 10.0f) % 10;

        float ab = barometricPressure < 0.0f ? -barometricPressure : barometricPressure;
        int b_i = (int)ab;
        int b_d = (int)(ab * 10.0f) % 10;
        snprintf(line, sizeof(line), "T %3d.%1d B %3d.%1d", t_i, t_d, b_i, b_d);
    }
    GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);

    {
        float aa = altitude < 0.0f ? -altitude : altitude;
        int a_i = (int)aa;
        int a_d = (int)(aa * 10.0f) % 10;
        snprintf(line, sizeof(line), "Alt %3d.%1d m", a_i, a_d);
    }
    GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);
}

void UI_Cube_Draw(UI_Widget* widget) {
    Point3D center = { 0.0f, 0.0f, -20.0f };    // closer to camera (try -20, -40, ...)
    Vector3D halfExtent = { 20.0f, 20.0f, 20.0f }; // larger half-size for clearer view

    // Build quaternion
    // Math3D_QuatFromEuler(yaw, pitch, roll) -- currently yaw=Z, pitch=Y, roll=X in our code
    // If rotation looks wrong, try swapping the order below (see alternative commented)
    Quaternion q = Math3D_QuatFromEuler(angleZ, angleY, angleX); // current mapping
    // Quaternion q = Math3D_QuatFromEuler(angleX, angleY, angleZ); // try if rotation axes swapped
    q = Math3D_QuatNormalize(q);

    GFX3D_DrawCube(&center, &halfExtent, &q, GFX_COLOR_WHITE);

    // animate rotation (tweak speeds if needed)
    angleX += 0.04f;
    angleY += 0.03f;
    angleZ += 0.02f;

    // show logic FPS
    char fpsLine[20];
    snprintf(fpsLine, sizeof(fpsLine), "FPS: %d", logicFps);
    GFX_DrawString(0, 0, fpsLine, GFX_COLOR_WHITE);
    // show screen FPS
    #ifndef SYNC_FPS_TO_SCREEN_REFRESH
    snprintf(fpsLine, sizeof(fpsLine), "SFPS: %d", screenFps);
    GFX_DrawString(0, 10, fpsLine, GFX_COLOR_WHITE);
    #endif
}
# endif

void System_Update_Task() {
    static uint8_t keyTiming = 0;
    static uint8_t lastRawStatus = KEY_STATE_RELEASED;
    
    // key handling with debounce
    if (keyEnable == KEY_ENABLE) {
        uint8_t currentRawStatus = !(GPIOA->IDR & GPIO_Pin_0);
        if (currentRawStatus == lastRawStatus) {
            if (++keyTiming >= KEY_DEBOUNCE_TIME) {
                if (keyStatus != currentRawStatus && keyStatus == KEY_STATE_RELEASED) {
                    keyPressCount++;
                }
                keyStatus = currentRawStatus;
                keyTiming = KEY_DEBOUNCE_TIME;
            }
        } else {
            keyTiming = 0;
            lastRawStatus = currentRawStatus;
        }
    }
    
    // LED handling
    static uint8_t ledTiming = 0;
    if (ledToggleCount > 0) {
        ledTiming++;
        if (ledTiming >= LED_TOGGLE_INTERVAL) {
            ledTiming = 0;

            if (ledToggleCmd & 0x01) {
              GPIOC->BSRRL = GPIO_Pin_13;
            } else {
              GPIOC->BSRRH = GPIO_Pin_13;
            }

            uint8_t lastBit = ledToggleCmd & 0x01;
            ledToggleCmd = (ledToggleCmd >> 1) | (lastBit << 7);

            ledToggleCount--;
        }
    } else {
        ledTiming = LED_TOGGLE_INTERVAL; 
    }

    // mpu data read
    Read_IMU_All();
    Read_BMP_All();
}


void Loop() {
    int logicCounter = 0;    // 程序循环计数
    int displayCounter = 0;  // 屏幕刷新计数
    int lastTime = sysTick;

    const uint16_t TARGET_FRAME_TIME = 30; // 帧间隔，单位毫秒 (10 FPS)
    uint16_t frameStart;
    
    while (1) {
        frameStart = sysTick;
        logicCounter++; // 每次循环增加程序 FPS 计数

        // 1. 核心系统任务更新 (按程序频率运行)
        System_Update_Task();

        // 2. 蓝牙命令处理
        uint8_t commandBuffer[RX_BUFFER_SIZE] = {0};
        uint16_t len = Read_Bluetooth_Command(commandBuffer);

        // if (len > 0) {
        //     # ifdef DISPLAY_ENABLE
        //     UI_Logger_AddLine(&logWindow, (char*)commandBuffer);
        //     # endif

        //     // ...existing command handling code...
        //     if (commandBuffer[0] == 'H') {
        //         currentState = STATE_HOME;
        //     } else if (commandBuffer[0] == 'C') {
        //         currentState = STATE_CUBE;
        //     } else if (commandBuffer[0] == 'N') {
        //         currentState = STATE_NONE;
        //     } else if (commandBuffer[0] == 'S') {
        //         currentState = STATE_MPU;
        //     } else {
        //         pwmDutyBuffer[0] = Map_Percent_To_Real(atoi((char*)commandBuffer));
        //         pwmDutyBuffer[1] = Map_Percent_To_Real(atoi((char*)commandBuffer));
        //         pwmDutyBuffer[2] = Map_Percent_To_Real(atoi((char*)commandBuffer));
        //         pwmDutyBuffer[3] = Map_Percent_To_Real(atoi((char*)commandBuffer));

        //         # ifdef DISPLAY_ENABLE
        //         uint8_t pwm_status[64];
        //         sprintf((char*)pwm_status, "PWM Set to: %d", Map_Percent_To_Real(atoi((char*)commandBuffer)));
        //         UI_Logger_AddLine(&logWindow, (char*)pwm_status);
        //         # endif
        //     }
        // }

        // 3. 渲染与屏幕刷新
        # ifdef DISPLAY_ENABLE
        GFX_Clear();
        UI_DrawTree(widgets[currentState], 0, 0);
        
        // 只有当 GFX_Update 成功启动刷新时，updateStarted 才为 1
        uint8_t updateStarted = GFX_Update();
        if (updateStarted) {
            displayCounter++; 
        }

        // 4. 每秒统计一次 FPS
        int now = sysTick;
        if (now - lastTime >= 1000) {
            screenFps = displayCounter; // 这里的 screenFps 现在代表屏幕实际刷新率
            logicFps = logicCounter;   // logicFps 代表程序逻辑频率
            displayCounter = 0;
            logicCounter = 0;
            lastTime = now;
        }
        # endif

        if (Key_PressConsume()) {
            char pwm_status[64];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
            Write_USART1_Data(pwm_status, strlen(pwm_status));


            # ifdef DISPLAY_ENABLE
            UI_Logger_AddLine(&logWindow, (char*)pwm_status);
            # endif

            currentState = STATE_MPU;


            // uint8_t clockSource = RCC_GetSYSCLKSource();
            // if (clockSource == 0x00) {
            //     // Write_USART1_Data("0", 30);
            // } else if (clockSource == 0x04) {
            //     // Write_USART1_Data("0", 30);
            // } else if (clockSource == 0x08) {
            //     // Write_USART1_Data("Clock Source: PLL\r\n", 19);
            //     // 如果是 PLL，需要进一步判断 PLL 的来源
            //     if (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE) {
            //         Write_USART1_Data("PLL Source: HSE\r\n", 17);
            //     } else {
            //         // Write_USART1_Data("PLL Source: HSI\r\n", 17);
            //     }
            // }
        }

        static int pwm_var = 0;
        if (pwm_var >= 50) pwm_var = 0;
        pwm_var += 10;
        pwmDutyBuffer[0] = Map_Percent_To_Real(pwm_var);
        pwmDutyBuffer[1] = Map_Percent_To_Real(pwm_var);
        pwmDutyBuffer[2] = Map_Percent_To_Real(pwm_var);
        pwmDutyBuffer[3] = Map_Percent_To_Real(pwm_var);


        // static char dida[16];
        // sprintf(dida, "0%d%d", 1, 0);
        // Write_USART1_Data(dida, strlen(dida));

        USART_SendData(USART1, 'A');
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

        uint16_t frameTime = sysTick - frameStart;
        if (frameTime < TARGET_FRAME_TIME) {
            delay_ms(TARGET_FRAME_TIME - frameTime);
        }
    }
}
