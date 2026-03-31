#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>




static float height_init, temperature_init;


/*-----------------------------------------------------------*/

static void exampleTask( void * parameters );

/*-----------------------------------------------------------*/

static void exampleTask( void * parameters )
{
    /* Unused parameters. */
    ( void ) parameters;

    for( ; ; ) {
        /* Example Task Code */
        vTaskDelay( 100 ); /* delay 100 ticks */

        // switch led
        static uint8_t ledState = 0;
        ledState = !ledState;
        if (ledState) {
            GPIO_LED->BSRRL = GPIO_LED_PIN;
        } else {
            GPIO_LED->BSRRH = GPIO_LED_PIN;
        }

    }
}
/*-----------------------------------------------------------*/


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
    // SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);  // 设置  SysTick  系统时钟中断为1mS

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // Key_GPIO_Init();
    LED_GPIO_Init();
    // Battery_GPIO_Init();
    // Battery_ADC_Init();
    // Battery_Measure_Reset();

    // LED_Blink(LED_TOGGLE_CMD_BLINK_FAST, 1);

    // OLED_GPIO_Init();
    // OLED_Init();
    // GFX_Init();
    // UI_Logger_Init(&logWindow, 0, 0, 127, 60);
    // Window_Init();

    // Init_DMA_For_USART1_RX(bleRxBuffer, sizeof(bleRxBuffer));
    // Init_DMA_For_USART1_TX(bleTxBuffer);
    // UART1_GPIO_Init();
    // BLE_Init(BLE_BAUDRATE_115200);
    // UI_Logger_AddLine(&logWindow, "UART Init OK");

    // SPI_Sensor_GPIO_Init();
    // SPI_Sensor_Init();
    // ICM20948_Init();
    // BMP280_Init();
    // Delay_ms(50);
    // // Init_DMA_For_IMU_SPI2_TIM2(imu_tx_buf, imu_rx_buf);
    // UI_Logger_AddLine(&logWindow, "Sensor Init OK");
    // uint8_t who_am_i = ICM20948_Read_WhoAmI();
    // uint8_t mag_who_am_i = ICM20948_Read_MagWhoAmI();
    // uint8_t bmp_who_am_i = BMP280_Read_WhoAmI();
    // char buf[64];
    // snprintf(buf, sizeof(buf), "ICM20948: 0x%02X", who_am_i);
    // UI_Logger_AddLine(&logWindow, buf);
    // snprintf(buf, sizeof(buf), "AK09916: 0x%02X", mag_who_am_i);
    // UI_Logger_AddLine(&logWindow, buf);
    // snprintf(buf, sizeof(buf), "BMP280: 0x%02X", bmp_who_am_i);
    // UI_Logger_AddLine(&logWindow, buf);
    // BMP280_Read(&height_init, &temperature_init);

    // sm_vec3_t accel_bias = {0.0f, 0.0f, 0.0f};
    // sm_vec3_t accel_scale = {1.0f, 1.0f, 1.0f};
    // uint8_t read = Persistence_ReadCalibData(-1, accel_bias, accel_scale);
    // Attitude_Init(accel_bias, accel_scale);
    // snprintf(buf, sizeof(buf), "Flash: %d", read);
    // UI_Logger_AddLine(&logWindow, buf);

    // PWM_GPIO_Init();
    // PWM_TIM_Init(PWM_PERIOD, PWM_PRESCALER);
    // Init_DMA_For_PWM_TIM3(pwmDutyBuffer);


    // Control_Init(RATE_LOOP_HZ);
    // UI_Logger_AddLine(&logWindow, "Control Init OK");

    // Command_SetModeCallback(Control_SetMode);
    // Command_SetThrottleCallback(Control_SetThrottle);
    // Command_SetHeightCallback(Control_SetHeight);
    // Command_MoveCallback(Control_Move);
    // Command_SetAttitudeCallback(Control_SetAttitude);
    // Command_ArmCallback(Control_Arm);
    // Command_EmergencyStopCallback(Control_EmergencyStop);
    // Command_FlightModeCallback(Control_FlightMode);

    TaskHandle_t xExampleTaskHandle = NULL;
    ( void ) xTaskCreate( exampleTask,
                        "example",
                        configMINIMAL_STACK_SIZE,
                        NULL,
                        configMAX_PRIORITIES - 1U,
                        &xExampleTaskHandle );


    /* Start the scheduler. */
    vTaskStartScheduler();

    for (;;) {
        /* Should not reach here. */

        // FPS_StartFrame();
        
        // float dt = FPS_GetDeltaTime();
        // Attitude_Update(dt);

        // Key_Toggle_Handler();
        // LED_Toggle_Handler();

        // ICM20948_Read(imu_rx_buf, mag_rx_buf);
        // BMP280_Read(&altitude_rx, &temperature_rx);

        // Battery_Measure_Step();

        // if (Key_PressConsume()) {
        //     if (Window_Current() == WINDOW_NONE) {
        //         Window_To(WINDOW_IMU);
        //     } else if (Window_Current() == WINDOW_IMU) {
        //         Window_To(WINDOW_CUBE);
        //     } else if (Window_Current() == WINDOW_CUBE) {
        //         Window_To(WINDOW_BATTERY);
        //     } else if (Window_Current() == WINDOW_BATTERY) {
        //         Window_To(WINDOW_PID);
        //     } else if (Window_Current() == WINDOW_PID) {
        //         Window_To(WINDOW_NONE);
        //     }
        // }

        // uint8_t buffer[12] = {0};
        // uint16_t len = 0;
        // if (bleRxStatusUart1 == BLE_RX_STATE_COMPLETE) {
        //     len = BLE_ReadData(buffer);
        //     Command_ParseAndExecute((char*)buffer, len);
        //     BLE_WriteData(buffer, len); // Echo back received data
        // }
        // if (len > 0) {
        //     UI_Logger_AddLine(&logWindow, (char*)buffer);
        // }

        // ControlAttitude_Loop();

        // char pwm_status[64];
        // sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
        //         TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
        // BLE_WriteData(pwm_status, strlen(pwm_status));


        // Window_Render();

        // FPS_EndFrame();
    }

}


/*-----------------------------------------------------------*/

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;

        // UI_Logger_AddLine(&logWindow, "Stack overflow detected!");

        taskDISABLE_INTERRUPTS();
        for( ; ; ) {
            /* Infinite loop to halt the system */
        }
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */
/*-----------------------------------------------------------*/
