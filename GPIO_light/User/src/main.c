#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


int main() {
    RCC_ClocksTypeDef RCC_Clocks;
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

    Key_GPIO_Init();
    LED_GPIO_Init();
    Battery_GPIO_Init();
    Battery_ADC_Init();
    Battery_Measure_Reset();

    LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 1);

    OLED_GPIO_Init();
    OLED_Init();
    GFX_Init();
    UI_Logger_Init(&logWindow, 0, 0, 127, 60);
    Window_Init();

    Init_DMA_For_USART1_RX(bleRxBuffer, sizeof(bleRxBuffer));
    Init_DMA_For_USART1_TX(bleTxBuffer);
    UART1_GPIO_Init();
    BLE_Init(BLE_BAUDRATE_115200);
    UI_Logger_AddLine(&logWindow, "UART Init OK");

    PWM_GPIO_Init();
    PWM_TIM_Init(PWM_PERIOD, PWM_PRESCALER);
    Init_DMA_For_PWM_TIM3(pwmDutyBuffer);
    UI_Logger_AddLine(&logWindow, "PWM Init OK");

    // Load_Bias_Quaternion_From_Flash(&q_bias);
    SPI_IMU_GPIO_Init();
    ICM20948_Init();
    BMP280_Init();
    Delay_ms(50);
    // Init_DMA_For_IMU_SPI2_TIM2(imu_tx_buf, imu_rx_buf);
    UI_Logger_AddLine(&logWindow, "IMU Init OK");
    uint8_t who_am_i = ICM20948_Read_WhoAmI();
    uint8_t mag_who_am_i = ICM20948_Read_MagWhoAmI();
    char buf[64];
    snprintf(buf, sizeof(buf), "ICM20948: 0x%02X", who_am_i);
    UI_Logger_AddLine(&logWindow, buf);
    snprintf(buf, sizeof(buf), "AK09916: 0x%02X", mag_who_am_i);
    UI_Logger_AddLine(&logWindow, buf);

    Attitude_Init();
    UI_Logger_AddLine(&logWindow, "Attitude Init OK");

    for (;;) {
        FPS_StartFrame();
        
        Attitude_Update(FPS_GetDeltaTime());

        Key_Toggle_Handler();
        LED_Toggle_Handler();

        ICM20948_Read(imu_rx_buf, mag_rx_buf);
        BMP280_Read();
        Battery_Measure_Step();

        if (Key_PressConsume()) {
            if (Window_Current() == WINDOW_NONE) {
                Window_To(WINDOW_IMU);
            } else if (Window_Current() == WINDOW_IMU) {
                Window_To(WINDOW_CUBE);
            } else if (Window_Current() == WINDOW_CUBE) {
                Window_To(WINDOW_BATTERY);
            } else {
                Window_To(WINDOW_NONE);
            }
        }

        uint8_t buffer[12] = {0};
        uint16_t len = 0;
        if (bleRxStatusUart1 == BLE_RX_STATE_COMPLETE) {
            len = BLE_ReadData(buffer);
            BLE_WriteData(buffer, len); // Echo back received data
        }
        if (len > 0) {
            UI_Logger_AddLine(&logWindow, (char*)buffer);
        }

        Window_Render();

        FPS_EndFrame();
    }

    // Init_Control();                               // 控制初始化函数
    // # ifdef DISPLAY_ENABLE
    // UI_Logger_AddLine(&logWindow, "Control Init OK");
    // # endif

}
