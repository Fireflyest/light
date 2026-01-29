#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

RCC_ClocksTypeDef RCC_Clocks;


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

    # ifdef DISPLAY_ENABLE
    UI_Logger_Init(&logWindow, 0, 0, 127, 60);
    # endif

    Init_USART(BAUDRATE_115200);                  // USART1 初始化函数
    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, "UART Init OK");
    # endif
    
    Init_PWM(PWM_PERIOD, PWM_PRESCALER);        // PWM 初始化函数
    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, "PWM Init OK");
    # endif

    Init_IMU();                               // IMU 初始化函数
    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, "IMU Init OK");
    # endif

    Init_Control();                               // 控制初始化函数
    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, "Control Init OK");
    # endif

    Init_Widgets();

    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, "Press Key to Start");
    # endif

    Start();
}
