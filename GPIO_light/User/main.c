#include "main.h"
#include <string.h>  // Include for standard memcpy
#include <stdio.h>   // Include for sprintf

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

    // 显示接收到的数据
    OLED_ShowString(0, 3, (u8*)"RX:");
    if (rxStatusUart1 == 1 && rxIndexUart1 > 0) {
        uint8_t buffer[11];
        uint16_t len = Read_USART1_Data(buffer);
        OLED_ShowString(24, 3, "          ");
        OLED_ShowString(24, 3, (u8*)buffer);
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
    Init_USART(9600);                  // USART1 初始化函数

    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);
    delay_ms(1000);

    I2C_Config();                               // I2C 初始化函数
    
    
    PWM_TIM_Config(PWM_PERIOD, PWM_PRESCALER);  // PWM 定时器配置初始化

    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    TIM_SetCompare3(TIM3, 0);
    TIM_SetCompare4(TIM3, 0);

    LED_Blink(LED_TOGGLE_CMD_BLINK_SLOW, 3);
    delay_ms(1000);

    Display_Init();  // OLED 显示屏初始化函数


    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 3);

    u16 pwm_val1 = 0;
    u16 pwm_val2 = 0;
    u16 pwm_val3 = 0;
    u16 pwm_val4 = 0;
    u8 direction = 1;              // 1:增加 0:减少
    u16 counter = 0;               // 计数器变量
    int max_pwm_val = PWM_PERIOD;  // 最大PWM值

    for (;;) {
        // OLED_ShowString(0, 2, (u8*)"STM32 Test");
        // delay_ms(1000);
        // Render_FrameBuffer();

        key = Key_Status();  // 得到键值
        if (key) {
            char pwm_status[50];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    pwm_val1, pwm_val2, pwm_val3, pwm_val4);
            Write_USART1_Data(pwm_status, strlen(pwm_status));
        }


        // if (uartStatus == 2 && rxIndex > 0) {
        //     // 解析命令：第一位选择设备，第二位设置PWM操作
        //     if (rxIndex >= 2) {  // 确保至少有2个字符
        //     int device = rxBuffer[0] - '0';  // 将第一位转换为数字
            
        //     // 检查第二位是数字、+、-
        //     int pwm_value = 0;
        //     if (rxBuffer[1] == '+') {
        //         switch (device) {
        //         case 1: pwm_val1 = (pwm_val1 < 2000) ? pwm_val1 + 50 : 2000; pwm_value = pwm_val1; break;
        //         case 2: pwm_val2 = (pwm_val2 < 2000) ? pwm_val2 + 50 : 2000; pwm_value = pwm_val2; break;
        //         case 3: pwm_val3 = (pwm_val3 < 2000) ? pwm_val3 + 50 : 2000; pwm_value = pwm_val3; break;
        //         case 4: pwm_val4 = (pwm_val4 < 2000) ? pwm_val4 + 50 : 2000; pwm_value = pwm_val4; break;
        //         }
        //     } else if (rxBuffer[1] == '-') {
        //         switch (device) {
        //         case 1: pwm_val1 = (pwm_val1 > 1000) ? pwm_val1 - 50 : 1000; pwm_value = pwm_val1; break;
        //         case 2: pwm_val2 = (pwm_val2 > 1000) ? pwm_val2 - 50 : 1000; pwm_value = pwm_val2; break;
        //         case 3: pwm_val3 = (pwm_val3 > 1000) ? pwm_val3 - 50 : 1000; pwm_value = pwm_val3; break;
        //         case 4: pwm_val4 = (pwm_val4 > 1000) ? pwm_val4 - 50 : 1000; pwm_value = pwm_val4; break;
        //         }
        //     } else {
        //         // 直接设置PWM值
        //         int pwm_level = rxBuffer[1] - '0';  // 将第二位转换为数字
                
        //         // 特殊情况：0代表2000
        //         if (pwm_level == 0) {
        //         pwm_value = 2000;
        //         } else {
        //         // 1-9映射到1100-1900
        //         pwm_value = 1000 + (pwm_level * 100);
        //         }
                
        //         // 根据设备选择设置相应的PWM通道
        //         switch (device) {
        //         case 1: pwm_val1 = pwm_value; break;
        //         case 2: pwm_val2 = pwm_value; break;
        //         case 3: pwm_val3 = pwm_value; break;
        //         case 4: pwm_val4 = pwm_value; break;
        //         }
        //     }
            
        //     // 确保PWM值不超过最大值
        //     pwm_val1 = (pwm_val1 > max_pwm_val) ? max_pwm_val : pwm_val1;
        //     pwm_val2 = (pwm_val2 > max_pwm_val) ? max_pwm_val : pwm_val2;
        //     pwm_val3 = (pwm_val3 > max_pwm_val) ? max_pwm_val : pwm_val3;
        //     pwm_val4 = (pwm_val4 > max_pwm_val) ? max_pwm_val : pwm_val4;
            
            
        //     // 更新LED状态
        //     LED2 = (pwm_value > 0) ? 0 : 1;
        //     }
        //     // 清空接收缓冲区
        //     memset(rxBuffer, 0, sizeof(rxBuffer));
        //     rxIndex = 0;
        //     uartStatus = 0;  // 恢复空闲状态
        // }



        // if (direction) {
        //     LED2 = 1;
        //     pwm_val++;
        //     if (pwm_val >= max_pwm_val) {
        //         UART_SendString(USART1, (u8*)"direction 0\r\n");
        //         direction = 0;
        //     }
        // } else {
        //     LED2 = 0;
        //     pwm_val--;
        //     if (pwm_val <= 950) {
        //         UART_SendString(USART1, (u8*)"direction 1\r\n");
        //         direction = 1;
        //     }
        // }
        TIM_SetCompare1(TIM3, pwm_val1);
        TIM_SetCompare2(TIM3, pwm_val2);
        TIM_SetCompare3(TIM3, pwm_val3);
        TIM_SetCompare4(TIM3, pwm_val4);

        show();  // 显示状态

        counter++;     // 计数器递增
        delay_ms(30);  // 适当延时
    }
}
