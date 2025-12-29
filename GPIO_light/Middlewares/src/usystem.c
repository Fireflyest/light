# include "usystem.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

__IO uint16_t sysTick;
UI_Logger logWindow;
uint16_t currentFps = 0;

typedef enum { 
    STATE_NONE,
    STATE_HOME,
    STATE_MPU,
    STATE_CUBE
} AppState;
AppState currentState;

UI_Widget* widgets[16];


static void UI_MPU_Draw(UI_Widget* widget);
static void UI_Cube_Draw(UI_Widget* widget);


void delay_ms(__IO uint32_t nTime) {
    uint32_t start = sysTick;
    while((sysTick - start) < nTime);
}


void Init_USystem() {
    Init_Key();
    Init_LED();
}

void Init_Display() {
    Init_OLED_Hardware();
    Init_GFX();
    Init_UI();
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
    Init_MPU_Hardware();
}

void Init_Widgets() {
    widgets[STATE_NONE] = (UI_Widget*)&logWindow;
    currentState = STATE_NONE;

    static UI_Window mpuWindow;
    UI_Window_Init(&mpuWindow, 0, 0, 128, 64);
    mpuWindow.base.draw = UI_MPU_Draw;
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
}

uint16_t Read_Bluetooth_Command(uint8_t* buffer) {
    uint16_t len = 0;
    if (rxStatusUart1) {
        len = Read_USART1_Data(buffer);
    }
    return len;
}


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

void UI_MPU_Draw(UI_Widget* widget) {
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
    float temp = (float)tempRaw / 333.87f + 21.0f;

    snprintf(line, sizeof(line), "AX %6d GX %6d", (int)ax, (int)gx);
    GFX_DrawString(x, y + 0 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AY %6d GY %6d", (int)ay, (int)gy);
    GFX_DrawString(x, y + 1 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "AZ %6d GZ %6d", (int)az, (int)gz);
    GFX_DrawString(x, y + 2 * lh, line, GFX_COLOR_WHITE);

    snprintf(line, sizeof(line), "T %5.1fC", (double)temp);
    GFX_DrawString(x, y + 3 * lh, line, GFX_COLOR_WHITE);
}

void UI_Cube_Draw(UI_Widget* widget) {
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

    angleX += 0.02f;
    angleY += 0.015f;
    angleZ += 0.01f;

    char fpsLine[20];
    snprintf(fpsLine, sizeof(fpsLine), "FPS: %d", currentFps);
    GFX_DrawString(0, 0, fpsLine, GFX_COLOR_WHITE);
}




void Loop() {

    int fpsCounter = 0;
    int lastTime = 0;

    const uint16_t TARGET_FRAME_TIME = 20; // 50 FPS, 20ms per frame
    // const uint16_t TARGET_FRAME_TIME = 33; // 30 FPS, 33ms per frame
    uint16_t frameStart;
    while (1) {
        frameStart = sysTick;
        GFX_Clear();

        int now = sysTick;
        fpsCounter++;
        if (now - lastTime >= 1000) {
            currentFps = fpsCounter;
            fpsCounter = 0;
            lastTime = now;
        }

        Read_MPU_All();

        uint8_t commandBuffer[RX_BUFFER_SIZE] = {0};
        uint16_t len = Read_Bluetooth_Command(commandBuffer);

        if (len > 0) {
            UI_Logger_AddLine(&logWindow, (char*)commandBuffer);

            if (commandBuffer[0] == 'H') {
                currentState = STATE_HOME;
            } else if (commandBuffer[0] == 'C') {
                currentState = STATE_CUBE;
            } else if (commandBuffer[0] == 'N') {
                currentState = STATE_NONE;
            } else if (commandBuffer[0] == 'S') {
                currentState = STATE_MPU;
            } else if (commandBuffer[0] == 'X') {

                uint8_t buf[14];
                char dbg[64];

                // 1) START
                uint32_t toi = 0x1FFFF;
                I2C_GenerateSTART(I2C2, ENABLE);
                while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && --toi);
                if(toi==0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT0"); continue; }

                // 2) ADDR (write) -> send register address
                toi = 0x1FFFF;
                I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Transmitter);
                while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --toi);
                if(toi==0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT1"); continue; }

                toi = 0x1FFFF;
                I2C_SendData(I2C2, MPU_ACCEL_XOUT_H);
                while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --toi);
                if(toi==0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT2"); continue; }

                // 3) Re-START and set to receiver
                toi = 0x1FFFF;
                I2C_GenerateSTART(I2C2, ENABLE);
                while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && --toi);
                if(toi==0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT3"); continue; }

                toi = 0x1FFFF;
                I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Receiver);
                while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --toi);
                if(toi==0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT4"); continue; }

                // 4) Read 14 bytes with correct ACK/STOP timing
                for (int i = 0; i < 14; ++i) {
                    // For 14-byte read, disable ACK before reading byte index 12 (i==12),
                    // then after reading idx 12 generate STOP so last byte is NACK'd by master.
                    if (i == 12) {
                        I2C_AcknowledgeConfig(I2C2, DISABLE);
                    }

                    toi = 0x1FFFF;
                    while(!(I2C2->SR1 & I2C_SR1_RXNE) && --toi);
                    if (toi == 0) {
                        UI_Logger_AddLine(&logWindow, "I2C TIMEOUT5");
                        // attempt to recover bus state
                        I2C_GenerateSTOP(I2C2, ENABLE);
                        I2C_AcknowledgeConfig(I2C2, ENABLE);
                        break;
                    }

                    buf[i] = I2C_ReceiveData(I2C2);

                    if (i == 12) {
                        // after reading the 13th byte, generate STOP to finish transfer properly
                        I2C_GenerateSTOP(I2C2, ENABLE);
                    }
                }

                // restore ACK for future reads
                I2C_AcknowledgeConfig(I2C2, ENABLE);

                // 5) report accel values
                snprintf(dbg, sizeof(dbg), "AX:%6d, AY:%6d, AZ:%6d",
                         (int16_t)((buf[0]<<8)|buf[1]),
                         (int16_t)((buf[2]<<8)|buf[3]),
                         (int16_t)((buf[4]<<8)|buf[5]));
                Write_USART1_Data((uint8_t*)dbg, strlen(dbg));
            } else if (commandBuffer[0] == 'M') {

                // Read_MPU_All();

                uint8_t regs[] = {MPU_PWR_MGMT_1, MPU_SMPLRT_DIV, MPU_CONFIG, MPU_GYRO_CONFIG, MPU_ACCEL_CONFIG};
                uint8_t whoami = 0xFF;
                uint8_t buf[14];
                uint8_t values[5];
                uint8_t timeout = 0xFF;
                char dbg[64];

                for(int i = 0; i < 5; i++) {
                    uint32_t toi;

                    // 1. 写寄存器地址 (写操作单步超时独立处理)
                    toi = 0xFFFF;
                    I2C_GenerateSTART(I2C2, ENABLE);
                    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT START"); values[i]=0xFF; continue; }

                    toi = 0xFFFF;
                    I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Transmitter);
                    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT ADDR-TX"); values[i]=0xFF; continue;}

                    toi = 0xFFFF;
                    I2C_SendData(I2C2, regs[i]);
                    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT TX-BYTE"); values[i]=0xFF; continue; }

                    // 2. 重复起始，准备读取 1 字节（单字节读：在发送从地址前关闭 ACK，以产生 NACK）
                    toi = 0xFFFF;
                    I2C_GenerateSTART(I2C2, ENABLE);
                    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT RSTART"); values[i]=0xFF; continue; }

                    // 对单字节读取：在地址为接收方向前，先关闭 ACK（保证接收的单字节被 NACK）
                    I2C_AcknowledgeConfig(I2C2, DISABLE);

                    toi = 0xFFFF;
                    I2C_Send7bitAddress(I2C2, MPU_ADDRESS, I2C_Direction_Receiver);
                    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT ADDR-RX"); values[i]=0xFF; 
                        // 恢复 ACK 以免影响后续多字节读取
                        I2C_AcknowledgeConfig(I2C2, ENABLE);
                        continue;
                    }

                    // 等待数据到达并读取
                    toi = 0xFFFF;
                    while(!(I2C2->SR1 & I2C_SR1_RXNE) && --toi);
                    if(toi == 0) { UI_Logger_AddLine(&logWindow, "I2C TIMEOUT RXNE"); values[i]=0xFF;
                        I2C_GenerateSTOP(I2C2, ENABLE);
                        I2C_AcknowledgeConfig(I2C2, ENABLE);
                        continue;
                    }
                    values[i] = I2C_ReceiveData(I2C2);

                    // 生成 STOP 并恢复 ACK（为后续多字节或下一次单字节读取准备）
                    I2C_GenerateSTOP(I2C2, ENABLE);
                    I2C_AcknowledgeConfig(I2C2, ENABLE);
                }

                snprintf(dbg, sizeof(dbg), "PWR:%02X DIV:%02X CFG:%02X GYR:%02X ACC:%02X\r\n",
                        values[0], values[1], values[2], values[3], values[4]);
                Write_USART1_Data((uint8_t*)dbg, strlen(dbg));
            } else {
                pwmDutyBuffer[0] = Map_Percent_To_Real(atoi((char*)commandBuffer));
                pwmDutyBuffer[1] = Map_Percent_To_Real(atoi((char*)commandBuffer));
                pwmDutyBuffer[2] = Map_Percent_To_Real(atoi((char*)commandBuffer));
                pwmDutyBuffer[3] = Map_Percent_To_Real(atoi((char*)commandBuffer));
                uint8_t pwm_status[64];
                sprintf((char*)pwm_status, "PWM Set to: %d", Map_Percent_To_Real(atoi((char*)commandBuffer)));
                UI_Logger_AddLine(&logWindow, (char*)pwm_status);
            }
        }

        UI_DrawTree(widgets[currentState], 0, 0);

        GFX_Update();



        if (Key_Status()) {
            char pwm_status[64];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
            Write_USART1_Data(pwm_status, strlen(pwm_status));
        }

        uint16_t frameTime = sysTick - frameStart;
        if (frameTime < TARGET_FRAME_TIME) {
            delay_ms(TARGET_FRAME_TIME - frameTime);
        }
    }
}

