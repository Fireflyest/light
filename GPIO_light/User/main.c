#define OLED_DEFINE_FONTS

#include "main.h"

RCC_ClocksTypeDef RCC_Clocks;

u8 key;  // 保存键值

void blink(u8 delay, u8 times) {
    for (u8 i = 0; i < times; i++) {
        LED2 = 0;
        delay_ms(delay);
        LED2 = 1;
        delay_ms(delay);
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

    GPIO_Config();  // I/O 引脚功能初始化函数 ： 初始化 LED 对应的引脚
    I2C_Configuration();  // I2C 初始化函数

    // 初始化OLED
    // OLEDConfiguration();
    // OLED_Display_Status(Display_ON);

    blink(50, 3);

    KEY_Init();  // 初始化与按键连接的硬件接口
    
    blink(100, 5);

    u16 counter = 0;  // 计数器变量
    char buffer[16];  // 字符缓冲区

    for (;;) {
        key = KEY_Scan(0);  // 得到键值

        if (key) {
            switch (key) {
            case K0_PRES:  // 按键K0按下
                LED2 = !LED2;  // LED状态翻转
                break;
            }
        } else {
            delay_ms(10);
        }

        // OLED_ShowString(0, 0, (u8*)"STM32 OLED Demo");
        
        // // 显示LED状态
        // OLED_ShowString(0, 1, (u8*)"LED:");
        // if (LED2 == 0)
        //     OLED_ShowString(32, 1, (u8*)"ON ");
        // else
        //     OLED_ShowString(32, 1, (u8*)"OFF");
            
        // // 显示计数器值
        // OLED_ShowString(0, 2, (u8*)"Counter:");

        // OLED_ShowString(56, 2, (u8*)buffer);
        
        // // 按键提示
        // OLED_ShowString(0, 3, (u8*)"Press K0 to toggle LED");
        
        counter++;  // 计数器递增
        delay_ms(100);  // 适当延时
    }
}
