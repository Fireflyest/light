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

void Init_IMU() {
    Init_IMU_Hardware();

    long sum[3] = {0, 0, 0};
    for(int i = 0; i < 100; i++) {
        Read_IMU_All();
        // 累加陀螺仪原始数据 (ICM-20948 对应索引 6-11)
        sum[0] += (int16_t)((mpuDataBuffer[6] << 8) | mpuDataBuffer[7]);
        sum[1] += (int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9]);
        sum[2] += (int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11]);
        delay_ms(5);
    }
    gyro_offset[0] = (float)sum[0] / 100.0f;
    gyro_offset[1] = (float)sum[1] / 100.0f;
    gyro_offset[2] = (float)sum[2] / 100.0f;

    Kalman_Init(&imu_ekf);
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
void UI_MPU_BMP_Draw(UI_Widget* widget) {
    char line[40];
    int x = widget->x + 2;
    int y = widget->y + 2;
    int lh = 10; // 行高，根据字体调整

    int16_t ax = (int16_t)((mpuDataBuffer[0] << 8) | mpuDataBuffer[1]);
    int16_t ay = (int16_t)((mpuDataBuffer[2] << 8) | mpuDataBuffer[3]);
    int16_t az = (int16_t)((mpuDataBuffer[4] << 8) | mpuDataBuffer[5]);

    #if defined(MPU_6500) || defined(MPU_9250)

    int16_t gx = (int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9]);
    int16_t gy = (int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11]);
    int16_t gz = (int16_t)((mpuDataBuffer[12] << 8) | mpuDataBuffer[13]);

    int16_t mx = (int16_t)((magDataBuffer[0] << 8) | magDataBuffer[1]);
    int16_t my = (int16_t)((magDataBuffer[2] << 8) | magDataBuffer[3]);
    int16_t mz = (int16_t)((magDataBuffer[4] << 8) | magDataBuffer[5]);

    int16_t tempRaw = (int16_t)((mpuDataBuffer[6] << 8) | mpuDataBuffer[7]);
    float tempC = (float)tempRaw / 333.87f + 21.0f;
    #endif // #if defined(MPU_6500) || defined(MPU_9250)

    #ifdef ICM_20948

    // ICM-20948 顺序: 加速度(0-5), 陀螺仪(6-11), 温度(12-13)
    int16_t gx = (int16_t)((mpuDataBuffer[6] << 8) | mpuDataBuffer[7]);
    int16_t gy = (int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9]);
    int16_t gz = (int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11]);

    int16_t mx = (int16_t)((magDataBuffer[0] << 8) | magDataBuffer[1]);
    int16_t my = (int16_t)((magDataBuffer[2] << 8) | magDataBuffer[3]);
    int16_t mz = (int16_t)((magDataBuffer[4] << 8) | magDataBuffer[5]);

    int16_t tempRaw = (int16_t)((mpuDataBuffer[12] << 8) | mpuDataBuffer[13]);
    float tempC = (float)tempRaw / 333.87f + 21.0f;
    #endif // #ifdef ICM_20948


    snprintf(line, sizeof(line), "AX %6d GX %6d", (int)ax, (int)gx);
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AY %6d GY %6d", (int)ay, (int)gy);
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AZ %6d GZ %6d", (int)az, (int)gz);
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);
    
    snprintf(line, sizeof(line), "MX %6d MY %6d", (int)mx, (int)my);
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);

    {
        float at = tempC < 0.0f ? -tempC : tempC;
        int ti = (int)at;                       // integer part
        int tf = (int)(at * 10.0f) % 10;         // one decimal
        if (tempC < 0.0f) {
            snprintf(line, sizeof(line), "MZ %6d  T -%d.%dC", (int)mz, ti, tf);
        } else {
            snprintf(line, sizeof(line), "MZ %6d  T %d.%dC", (int)mz, ti, tf);
        }
    }
    GFX_DrawString(x, y + 4 * lh, line, GFX_COLOR_WHITE);


    {
        float at = temperature < 0.0f ? -temperature : temperature;
        int t_i = (int)at;
        int t_d = (int)(at * 10.0f) % 10;

        float ab = barometricPressure < 0.0f ? -barometricPressure : barometricPressure;
        int b_i = (int)ab;
        int b_d = (int)(ab * 10.0f) % 10;

        float aa = altitude < 0.0f ? -altitude : altitude;
        int a_i = (int)aa;
        int a_d = (int)(aa * 10.0f) % 10;
        snprintf(line, sizeof(line), "%3d.%1d %3d.%1d %3d.%1dm", t_i, t_d, b_i, b_d, a_i, a_d);
    }
    GFX_DrawString(x, y + 5 * lh, line, GFX_COLOR_WHITE);
    GFX_DrawString(x, y + 6 * lh, " ", GFX_COLOR_WHITE);
}

void UI_Cube_Draw(UI_Widget* widget) {
    Point3D center = { 0.0f, 0.0f, -20.0f };    // closer to camera (try -20, -40, ...)
    Vector3D halfExtent = { 20.0f, 20.0f, 20.0f }; // larger half-size for clearer view

    Quaternion q;
    q.w = imu_ekf.q[0];
    q.x = -imu_ekf.q[1]; // 取负号即为共轭 (Inverse rotation)
    q.y = -imu_ekf.q[2];
    q.z = -imu_ekf.q[3];

    GFX3D_DrawCube(&center, &halfExtent, &q, GFX_COLOR_WHITE);

    float axisLen = 10.0f; // 轴的长度（应大于立方体半长 20.0f）
    Vector3D vX = { axisLen, 0.0f, 0.0f };
    Vector3D vY = { 0.0f, axisLen, 0.0f };
    Vector3D vZ = { 0.0f, 0.0f, axisLen * 2 };

    // 使用四元数旋转轴向量
    Math3D_QuatRotateVector(&vX, &q);
    Math3D_QuatRotateVector(&vY, &q);
    Math3D_QuatRotateVector(&vZ, &q);

    // 计算三轴末端在 3D 空间的位置
    Point3D pStart, pEnd;

    // X轴
    pStart = center;
    pEnd.x = center.x + vX.x; pEnd.y = center.y + vX.y; pEnd.z = center.z + vX.z;
    GFX3D_DrawLine(&pStart, &pEnd, GFX_COLOR_WHITE);

    // Y轴
    pStart = center;
    pEnd.x = center.x + vY.x; pEnd.y = center.y + vY.y; pEnd.z = center.z + vY.z;
    GFX3D_DrawLine(&pStart, &pEnd, GFX_COLOR_WHITE);

    // Z轴
    pStart = center;
    pEnd.x = center.x + vZ.x; pEnd.y = center.y + vZ.y; pEnd.z = center.z + vZ.z;
    GFX3D_DrawLine(&pStart, &pEnd, GFX_COLOR_WHITE);

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
}


void Loop() {
    uint16_t logicCounter = 0;    // 程序循环计数
    uint16_t displayCounter = 0;  // 屏幕刷新计数
    uint16_t lastTime = sysTick;
    uint16_t lastUpdateTick = sysTick;

    const uint16_t TARGET_FRAME_TIME = 10; // 帧间隔
    uint16_t frameStart;
    
    while (1) {
        frameStart = sysTick;
        logicCounter++; // 每次循环增加程序 FPS 计数

        // 1. 核心系统任务更新 (按程序频率运行)
        System_Update_Task();

        float dt = (uint16_t)(frameStart - lastUpdateTick) / 1000.0f;
        lastUpdateTick = frameStart;
        Read_IMU_All();
        Read_BMP_All();
        Attitude_Update(dt); 


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

            currentState = STATE_CUBE;


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
