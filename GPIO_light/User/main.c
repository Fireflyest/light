#include "main.h"
#include <string.h>  // Include for standard memcpy
#include <stdio.h>   // Include for sprintf
#include <stdlib.h>

RCC_ClocksTypeDef RCC_Clocks;

u8 key;           // 保存键值


void show(void) {
    u8 buffer[16];  // 字符缓冲区

    OLED_ShowString(0, 0, (u8*)"STM32 UART Test");

    // 显示LED状态
    OLED_ShowString(0, 1, (u8*)"LED:");
    if (LED2 == 0)
        OLED_ShowString(32, 1, (u8*)"ON ");
    else
        OLED_ShowString(32, 1, (u8*)"OFF");

    // 显示UART状态
    OLED_ShowString(0, 2, (u8*)"UART:");
    switch (rxStatusUart1) {
    case 0:
        OLED_ShowString(40, 2, (u8*)"IDLE  ");
        break;
    case 1:
        OLED_ShowString(40, 2, (u8*)"RX OK ");
        break;
    }

    // 显示全部PWM状态
    char pwmStatus[64];
    sprintf(pwmStatus, "B0:%4d", pwmDutyBuffer[0]);
    OLED_ShowString(0, 4, pwmStatus);

    // 显示接收到的数据
    OLED_ShowString(0, 3, (u8*)"RX:");
    if (rxStatusUart1 == 1 && rxIndexUart1 > 0) {
        uint8_t buffer[11];
        uint16_t len = Read_USART1_Data(buffer);
        OLED_ShowString(24, 3, "          ");
        OLED_ShowString(24, 3, (u8*)buffer);
        // 转数字 0~100 每5变化
        pwmDutyBuffer[0] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[1] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[2] = Map_Percent_To_Real(atoi((char*)buffer));
        pwmDutyBuffer[3] = Map_Percent_To_Real(atoi((char*)buffer));
    }
}

void Display_Init(void) {
    // 初始化OLED
    OLEDConfiguration();
    Render_Init();

    // Render_Test();

    // // 初始化UI系统
    // UIWindow mainWindow;
    // UI_CreateWindow(&mainWindow, 10, 10, 116, 54, (u8*)"Main", LINE_SOLID);
    // UI_Init(&mainWindow);

    // // 创建主窗口内的组件
    // UILabel titleLabel;
    // UI_CreateLabel(&titleLabel, 0, 20, 60, 8, (u8*)"STM32 Demo UI");
    // titleLabel.alignment = ALIGN_CENTER;

    // UIButton button1;
    // UI_CreateButton(&button1, 0, 40, 60, 12, (u8*)"LED Control", LINE_SOLID);
    // button1.onClick = 0;

    // UICheckbox* checkbox1 = UI_CreateCheckbox(25, 40, 78, 8, "Enable Feature", 0);
    // checkbox1->onToggle = 0;

    // UIProgressBar* progress1 = UI_CreateProgressBar(20, 55, 88, 6, 75, 0);
    // progress1->progress = 75;  // 75%

    // // 添加组件到界面
    // UI_AddChild(g_uiManager.root, (UIObject*)&titleLabel);
    // UI_AddChild(g_uiManager.root, (UIObject*)&button1);
    // UI_AddChild(g_uiManager.root, (UIObject*)&checkbox1);
    // UI_AddChild(g_uiManager.root, (UIObject*)&progress1);

    // 设置初始焦点
    // UI_SetFocus((UIObject*)button1);

    // 刷新屏幕
    // UI_RefreshScreen();
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

    GPIO_Config();  // I/O 引脚功能初始化函数 ： 初始化 LED 对应的引脚
    Init_USystem();                           // 系统初始化函数
    Init_USART(BUADRATE_9600);                  // USART1 初始化函数


    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);
    delay_ms(1000);

    I2C_Config();                               // I2C 初始化函数
     
    Init_PWM(PWM_PERIOD, PWM_PRESCALER);        // PWM 初始化函数

    LED_Blink(LED_TOGGLE_CMD_BLINK_SLOW, 3);
    delay_ms(1000);

    Display_Init();  // OLED 显示屏初始化函数


    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);

    for (;;) {
        // OLED_ShowString(0, 2, (u8*)"STM32 Test");
        // delay_ms(1000);
        // Render_FrameBuffer();

        key = Key_Status();  // 得到键值
        if (key) {
            char pwm_status[64];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    pwmDutyBuffer[0], pwmDutyBuffer[1], pwmDutyBuffer[2], pwmDutyBuffer[3]);
            Write_USART1_Data(pwm_status, strlen(pwm_status));
        }



        show();  // 显示状态

        delay_ms(30);  // 适当延时
    }
}
