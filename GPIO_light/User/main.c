#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

RCC_ClocksTypeDef RCC_Clocks;

// 3D Cube Data
Point3D cubeVertices[8] = {
    {-10, -10, -10}, {10, -10, -10}, {10, 10, -10}, {-10, 10, -10},
    {-10, -10, 10}, {10, -10, 10}, {10, 10, 10}, {-10, 10, 10}
};

int cubeEdges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Front face
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Back face
    {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Connecting lines
};

float angleX = 0, angleY = 0, angleZ = 0;

void DrawCube(void) {
    Point2D projected[8];
    
    for(int i=0; i<8; i++) {
        Point3D p = cubeVertices[i];
        p = Math3D_RotateX(p, angleX);
        p = Math3D_RotateY(p, angleY);
        p = Math3D_RotateZ(p, angleZ);
        projected[i] = Math3D_Project(p, 64, 40); // Focal length 64, Camera Z 40
    }
    
    for(int i=0; i<12; i++) {
        Point2D p1 = projected[cubeEdges[i][0]];
        Point2D p2 = projected[cubeEdges[i][1]];
        GFX_DrawLine(p1.x, p1.y, p2.x, p2.y, GFX_COLOR_WHITE);
    }
}

void show(void) {
    GFX_DrawString(0, 0, "STM32 3D & UART", GFX_COLOR_WHITE);

    // UART Status
    GFX_DrawString(0, 20, "UART:", GFX_COLOR_WHITE);
    switch (rxStatusUart1) {
    case 0:
        GFX_DrawString(40, 20, "IDLE  ", GFX_COLOR_WHITE);
        break;
    case 1:
        GFX_DrawString(40, 20, "RX OK ", GFX_COLOR_WHITE);
        break;
    }

    // PWM Status
    char pwmStatus[64];
    sprintf(pwmStatus, "B0:%4d", pwmDutyBuffer[0]);
    GFX_DrawString(0, 40, pwmStatus, GFX_COLOR_WHITE);

    // RX Data
    GFX_DrawString(0, 30, "RX:", GFX_COLOR_WHITE);
    if (rxStatusUart1 == 1 && rxIndexUart1 > 0) {
        uint8_t buffer[11];
        uint16_t len = Read_USART1_Data(buffer);
        GFX_DrawString(24, 30, (char*)buffer, GFX_COLOR_WHITE);
        
        // Logic from original show()
        pwmDutyBuffer[0] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[1] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[2] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[3] = Map_Percent_To_Real(atoi((char*)buffer));
    }
}


int main() {
    /* Enable Clock Security System(CSS): this will generate an NMI exception
     when HSE clock fails *****************************************************/
    RCC_ClockSecuritySystemCmd(ENABLE);

    /*!< At this stage the microcontroller clock setting is already configured,
        this is done through SystemInit() function which is called from startup
        files before to branch to application main.
        To reconfigure the default setting of SystemInit() function,
        refer to system_stm32f4xx.c file */

    /* SysTick end of count event each 1ms */
    SystemCoreClockUpdate();                           // 更新  RCC_Clocks 系统时钟变量
    RCC_GetClocksFreq(&RCC_Clocks);                    // 获取  RCC_Clocks  系统时钟
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);  // 设置  SysTick  系统时钟中断为1mS

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    Init_USystem();                           // 系统初始化函数
    Init_Display();  // OLED 显示屏初始化函数

    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);

    // UI Setup
    typedef enum { STATE_STARTUP, STATE_COUNTDOWN, STATE_CUBE } AppState;
    AppState currentState = STATE_STARTUP;
    
    UI_TextList logWindow;
    UI_TextList_Init(&logWindow, 0, 0, 127, 60);
    UI_TextList_AddLine(&logWindow, "System Init OK");
    UI_TextList_AddLine(&logWindow, "Initing UART...");

    Init_USART(BUADRATE_9600);                  // USART1 初始化函数

    UI_TextList_AddLine(&logWindow, "UART Init OK");
    UI_TextList_AddLine(&logWindow, "Initing PWM...");

    Init_PWM(PWM_PERIOD, PWM_PRESCALER);        // PWM 初始化函数

    UI_TextList_AddLine(&logWindow, "PWM Init OK");

  
    int countdownValue = 3;
    int frameCount = 0;
    char countdownText[16];
    
    UI_Label lblTitle, lblCount;
    UI_Label_Init(&lblTitle, 10, 10, "Starting in:");
    UI_Label_Init(&lblCount, 50, 30, countdownText);

    for (;;) {
        GFX_Clear();

        if (currentState == STATE_COUNTDOWN) {
            // Update Countdown Logic
            frameCount++;
            if (frameCount >= 33) { // Approx 1 second (30ms * 33 = 990ms)
                frameCount = 0;
                countdownValue--;
                if (countdownValue < 0) {
                    currentState = STATE_CUBE;
                }
            }
            
            // Draw UI
            sprintf(countdownText, "%d", countdownValue);
            UI_DrawTree((UI_Widget*)&lblTitle, 0, 0);
            UI_DrawTree((UI_Widget*)&lblCount, 0, 0);
            
            // Draw a simple box around the countdown
            GFX_DrawRect(5, 5, 118, 54, GFX_COLOR_WHITE);
            
        } else if (currentState == STATE_CUBE) {
            // 3D Logic
            angleX += 0.05;
            angleY += 0.03;
            angleZ += 0.01;
            DrawCube();
            
            // Optional: Show status overlay
            // show();
        } else if (currentState == STATE_STARTUP) {
            // 显示日志窗口
            UI_DrawTree((UI_Widget*)&logWindow, 0, 0);
        }

        // Render
        GFX_Update();

        if (Key_Status()) {
            currentState = STATE_COUNTDOWN;
            char pwm_status[64];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    pwmDutyBuffer[0], pwmDutyBuffer[1], pwmDutyBuffer[2], pwmDutyBuffer[3]);
            Write_USART1_Data(pwm_status, strlen(pwm_status));
        }

        delay_ms(30);  // 适当延时
    }
}
