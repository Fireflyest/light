#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

RCC_ClocksTypeDef RCC_Clocks;

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

    Init_USystem();  // 按键和LED灯初始化
    Init_Display();  // OLED 显示屏初始化函数
    
    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);

    UI_Logger_Init(&logWindow, 0, 0, 127, 60);

    Init_USART(BUADRATE_9600);                  // USART1 初始化函数

    UI_Logger_AddLine(&logWindow, "UART Init OK");

    Init_PWM(PWM_PERIOD, PWM_PRESCALER);        // PWM 初始化函数

    UI_Logger_AddLine(&logWindow, "PWM Init OK");

    Init_MPU();                               // MPU6050 初始化函数

    UI_Logger_AddLine(&logWindow, "MPU6050 Init OK");

    Init_Widgets();

    UI_Logger_AddLine(&logWindow, "Press Key to Start");

    Loop(&logWindow);
}
