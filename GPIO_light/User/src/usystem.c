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
static void UI_IMU_Draw(UI_Widget* widget);
static void UI_Cube_Draw(UI_Widget* widget);
static void UI_Pid_Draw(UI_Widget* widget);
static void UI_Accel_Calib_Draw(UI_Widget* widget);
static void UI_Gyro_Calib_Draw(UI_Widget* widget);
static void UI_Mag_Calib_Draw(UI_Widget* widget);
static void UI_RenderFrame(void);

typedef enum { 
    STATE_NONE,
    STATE_HOME,
    STATE_MPU,
    STATE_PID,
    STATE_ACCEL_CALIBRATE,
    STATE_GYRO_CALIBRATE,
    STATE_MAG_CALIBRATE,
    STATE_CUBE
} AppState;
AppState currentState;
# endif

uint16_t logicCounter = 0;    // 程序循环计数
uint16_t displayCounter = 0;  // 屏幕刷新计数
uint16_t lastTime = 0;
uint16_t lastUpdateTick = 0;
uint16_t frameStart;
float dt;
const uint16_t TARGET_FRAME_TIME = 10; // 帧间隔ms

typedef enum {
    CALIB_WAIT_BUTTON,
    CALIB_STABILIZING,
    CALIB_ACCEL_COLLECTING,
    CALIB_DONE
} CalibState;

static CalibState accelCalibState, gyroCalibState, magCalibState = CALIB_WAIT_BUTTON;
static int accelCalibStep = 0; // 0-5 for 6 poses
static int stableCount = 0;
static int collectCount = 0;
static float accelData[6][3]; // 6 poses, 3 axes
static float gyroData[3]; // sum for bias
static int gyroSamples = 0;
static float magData[1000][3]; // sum for hard iron
static int magSamples = 0;

__IO uint32_t marker = 1;
static void Save_Bias_Quaternion_To_Flash(Quaternion* q);
static void Load_Bias_Quaternion_From_Flash(Quaternion* q);
Quaternion q_bias = {1.0f, 0.0f, 0.0f, 0.0f};
#define FLASH_BIAS_ADDR  ((uint32_t)0x0803E000)


void delay_ms(__IO uint32_t nTime) {
    uint32_t start = sysTick;
    while((sysTick - start) < nTime);
}


void Init_USystem() {
    Key_GPIO_Init();
    LED_GPIO_Init();
    Battery_GPIO_Init();
    Battery_ADC_Init();
}

void Init_Display() {
    # ifdef DISPLAY_ENABLE
    OLED_GPIO_Init();
    OLED_Init();
    GFX_Init();
    UI_Init();
    # endif
}

void Init_USART(uint32_t baudrate) {
    Init_DMA_For_USART1_RX(rxBufferUart1, sizeof(rxBufferUart1));
    Init_DMA_For_USART1_TX(txBufferUart1);
    UART1_GPIO_Init();
    Init_USART1(baudrate);
}

void Init_PWM(uint16_t period, uint16_t prescaler) {
    PWM_GPIO_Init();
    PWM_TIM_Init(period, prescaler);
    Init_DMA_For_PWM_TIM3(pwmDutyBuffer);
}

void Init_IMU() {
    // 先初始化两个传感器 (ICM20948_Init, BMP280_Init)
    // 最后再调用 Init_DMA_For_IMU_SPI2_TIM2 和 TIM_Config_For_IMU()
    Load_Bias_Quaternion_From_Flash(&q_bias);
    Init_IMU_Hardware();
    delay_ms(50);
    Attitude_Kalman_Init(&imu_ekf);
    Locate_Kalman_Init(&loc_ekf);
}

void Init_Control() {
    PID_Init(&pidRoll, 4.0f, 0.0f, 0.2f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidPitch, 4.0f, 0.0f, 0.2f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidYaw, 2.0f, 0.0f, 0.1f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidHeight, 1.0f, 0.0f, 0.1f, -100.0f, 100.0f, 0.02f, -500.0f, 500.0f, 1.0f);
    PID_Init(&pidRateRoll, 0.15f, 0.001f, 0.002f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.15f, 0.001f, 0.002f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);
    PID_Init(&pidRateYaw, 0.10f, 0.0005f, 0.001f, -50.0f, 50.0f, 0.01f, -400.0f, 400.0f, 1.0f);

    // 启动内环定时器（1kHz）
    RateControl_Init(RATE_LOOP_HZ);
}

void Init_Widgets() {
    # ifdef DISPLAY_ENABLE
    widgets[STATE_NONE] = (UI_Widget*)&logWindow;
    currentState = STATE_NONE;

    static UI_Window mpuWindow;
    UI_Window_Init(&mpuWindow, 0, 0, 128, 64);
    mpuWindow.base.draw = UI_IMU_Draw;
    widgets[STATE_MPU] = (UI_Widget*)&mpuWindow;

    static UI_Window pidWindow;
    UI_Window_Init(&pidWindow, 0, 0, 128, 64);
    pidWindow.base.draw = UI_Pid_Draw;
    widgets[STATE_PID] = (UI_Widget*)&pidWindow;

    static UI_Window homeWindow;
    UI_Window_Init(&homeWindow, 0, 0, 128, 64);
    static UI_Label lblTitle, lblCount;
    UI_Label_Init(&lblTitle, 10, 10, "Home:");
    UI_Label_Init(&lblCount, 50, 30, "aaaaaa");
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblTitle);
    UI_AddChild((UI_Widget*)&homeWindow, (UI_Widget*)&lblCount);
    widgets[STATE_HOME] = (UI_Widget*)&homeWindow;


    static UI_Window accelCalibWindow;
    UI_Window_Init(&accelCalibWindow, 0, 0, 128, 64);
    accelCalibWindow.base.draw = UI_Accel_Calib_Draw;
    widgets[STATE_ACCEL_CALIBRATE] = (UI_Widget*)&accelCalibWindow;

    static UI_Window gyroCalibWindow;
    UI_Window_Init(&gyroCalibWindow, 0, 0, 128, 64);
    gyroCalibWindow.base.draw = UI_Gyro_Calib_Draw;
    widgets[STATE_GYRO_CALIBRATE] = (UI_Widget*)&gyroCalibWindow;

    static UI_Window magCalibWindow;
    UI_Window_Init(&magCalibWindow, 0, 0, 128, 64);
    magCalibWindow.base.draw = UI_Mag_Calib_Draw;
    widgets[STATE_MAG_CALIBRATE] = (UI_Widget*)&magCalibWindow;


    static UI_Window cubeWindow;
    UI_Window_Init(&cubeWindow, 0, 0, 128, 64);
    cubeWindow.base.draw = UI_Cube_Draw;
    widgets[STATE_CUBE] = (UI_Widget*)&cubeWindow;
    # endif
}

void Parse_Bluetooth_Command() {
    uint8_t buffer[RX_BUFFER_SIZE] = {0};
    uint16_t len = 0;
    if (rxStatusUart1) {
        len = Read_USART1_Data(buffer);
    }
    // if (len > 0) {
    //     # ifdef DISPLAY_ENABLE
    //     UI_Logger_AddLine(&logWindow, (char*)commandBuffer);
    //     # endif

    //     // ...existing command handling code...
    //     if (buffer[0] == 'H') {
    //         currentState = STATE_HOME;
    //     } else if (buffer[0] == 'C') {
    //         currentState = STATE_CUBE;
    //     } else if (buffer[0] == 'N') {
    //         currentState = STATE_NONE;
    //     } else if (buffer[0] == 'S') {
    //         currentState = STATE_MPU;
    //     } else {
    //         pwmDutyBuffer[0] = PWM_Map_Percent(atoi((char*)buffer));
    //         pwmDutyBuffer[1] = PWM_Map_Percent(atoi((char*)buffer));
    //         pwmDutyBuffer[2] = PWM_Map_Percent(atoi((char*)buffer));
    //         pwmDutyBuffer[3] = PWM_Map_Percent(atoi((char*)buffer));

    //         # ifdef DISPLAY_ENABLE
    //         uint8_t pwm_status[64];
    //         sprintf((char*)pwm_status, "PWM Set to: %d", PWM_Map_Percent(atoi((char*)buffer)));
    //         UI_Logger_AddLine(&logWindow, (char*)pwm_status);
    //         # endif
    //     }
    // }
}



# ifdef DISPLAY_ENABLE
void UI_IMU_Draw(UI_Widget* widget) {
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

static float pidErrorHistory[3][128] = {0}; // 0:roll, 1:pitch, 2:yaw
static uint16_t pidErrorLen = 0;
void UI_Pid_Draw(UI_Widget* widget) {

    UI_Cube_Draw(widget); // 先画立方体作为背景参考

    int x0 = widget->x + 2;
    int y0 = widget->y + widget->h / 2; // 中线
    int w = widget->w - 4;
    int h = widget->h - 4;

    // 获取当前误差
    float errors[3] = {
        (rateSetRoll - gyroFilt[0].output), 
        (rateSetPitch - gyroFilt[1].output), 
        (rateSetYaw - gyroFilt[2].output)
    };

    // 更新历史
    if (pidErrorLen < w) {
        for (int i = 0; i < 3; i++)
            pidErrorHistory[i][pidErrorLen] = errors[i];
        pidErrorLen++;
    } else {
        // 左移一格
        for (int i = 0; i < 3; i++)
            memmove(&pidErrorHistory[i][0], &pidErrorHistory[i][1], (w - 1) * sizeof(float));
        for (int i = 0; i < 3; i++)
            pidErrorHistory[i][w - 1] = errors[i];
    }

    static float globalScale = 0.1f; // 初始可视化幅度（单位与误差一致）
    float curMax = 1e-6f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < pidErrorLen; j++)
            if (fabsf(pidErrorHistory[i][j]) > curMax)
                curMax = fabsf(pidErrorHistory[i][j]);

    // 快速增长，缓慢衰减（调整下方因子以改变响应/回落速度）
    if (curMax > globalScale) {
        globalScale = curMax;
    } else {
        globalScale *= 0.995f; // 0.995 => 0.5x 需要约 ln(0.5)/ln(0.995) 帧数
        if (globalScale < 1e-3f) globalScale = 1e-3f; // 下限保护
    }

    float scale = globalScale;

    // 画三条线（逐点画像素）
    for (int i = 0; i < 3; i++) {
        uint16_t color = GFX_COLOR_WHITE;
        int lastY = y0 - (int)(pidErrorHistory[i][0] / scale * (h / 2));
        for (int j = 1; j < pidErrorLen; j++) {
            int y = y0 - (int)(pidErrorHistory[i][j] / scale * (h / 2));
            GFX_Line_Style style = i == 0 ? GFX_LINE_STYLE_THICK_DOT :
                                  (i == 1 ? GFX_LINE_STYLE_ALTERNATE_2_1PX : GFX_LINE_STYLE_SINGLE_PIXEL);
            GFX_DrawLineStyled(x0 + j - 1, lastY, x0 + j, y, color, style);
            lastY = y;
        }
    }

    // 画左侧中线原点
    GFX_DrawPixel(x0, y0, GFX_COLOR_WHITE);
}

void UI_Cube_Draw(UI_Widget* widget) {
    Point3D center = { 0.0f, 0.0f, -20.0f };    // closer to camera (try -20, -40, ...)
    Vector3D halfExtent = { 20.0f, 20.0f, 20.0f }; // larger half-size for clearer view

    Quaternion q;
    q.w = imu_ekf.q_corr[0];
    q.x = -imu_ekf.q_corr[1]; // 取负号即为共轭 (Inverse rotation)
    q.y = -imu_ekf.q_corr[2];
    q.z = -imu_ekf.q_corr[3];

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

    // show altitude
    char altLine[20];
    snprintf(altLine, sizeof(altLine), "Alt: %dm", (int) loc_ekf.h);
    GFX_DrawString(72, 0, altLine, GFX_COLOR_WHITE);

}

static void UI_Accel_Calib_Draw(UI_Widget* widget) {
    static uint8_t accelCalibDone = 0;
    static int g_int;
    GFX_DrawString(2, 2, "Accel Calib", GFX_COLOR_WHITE);
    char stepStr[32];
    if (accelCalibStep < 6) {
        snprintf(stepStr, sizeof(stepStr), "Step %d/6: ", accelCalibStep + 1);
        GFX_DrawString(2, 12, stepStr, GFX_COLOR_WHITE);
        if (accelCalibState == CALIB_WAIT_BUTTON) {
            GFX_DrawString(2, 22, "Press key to start", GFX_COLOR_WHITE);
        } else if (accelCalibState == CALIB_STABILIZING) {
            GFX_DrawString(2, 22, "Stabilizing...", GFX_COLOR_WHITE);
        } else {
            GFX_DrawString(2, 22, "Collecting...", GFX_COLOR_WHITE);
        }
    } else if (!accelCalibDone) {
        GFX_DrawString(2, 12, "Calib Collect Done!", GFX_COLOR_WHITE);
        GFX_DrawString(2, 12, "Calib Complete...", GFX_COLOR_WHITE);
        float bias[3] = {0};
        float scale[3] = {1.0f, 1.0f, 1.0f}; // per axis
        // 计算动态 g（平均合力大小）
        float g = 0;
        for (int i = 0; i < 6; i++) {
            float norm = sqrtf(accelData[i][0]*accelData[i][0] + accelData[i][1]*accelData[i][1] + accelData[i][2]*accelData[i][2]);
            g += norm;
        }
        g /= 6.0f;
        g_int = (int)(g + 0.5f);
        const int max_iter = 10;
        const float alpha = 0.01f; // 学习率
        for (int iter = 0; iter < max_iter; iter++) {
            float grad_bias[3] = {0};
            float grad_scale[3] = {0};
            for (int i = 0; i < 6; i++) {
                float a_corr[3];
                for (int j = 0; j < 3; j++) a_corr[j] = (accelData[i][j] - bias[j]) / scale[j];
                float norm = sqrtf(a_corr[0]*a_corr[0] + a_corr[1]*a_corr[1] + a_corr[2]*a_corr[2]);
                if (norm > 1e-6f) {
                    float error = norm - g;
                    for (int j = 0; j < 3; j++) grad_bias[j] += error * a_corr[j] / (norm * scale[j]);
                    for (int j = 0; j < 3; j++) grad_scale[j] += error * norm / scale[j];
                }
            }
            // 更新参数
            for (int j = 0; j < 3; j++) bias[j] -= alpha * grad_bias[j];
            for (int j = 0; j < 3; j++) scale[j] -= alpha * grad_scale[j];
        }
        // 存储到 imuCalibData
        for (int j = 0; j < 3; j++) imuCalibData.accel_bias[j] = bias[j];
        for (int j = 0; j < 3; j++) imuCalibData.accel_scale[j] = scale[j];
        GFX_DrawString(2, 22, "Bias & Scale saved", GFX_COLOR_WHITE);
        accelCalibDone = 1;
    } else {
        char verify[32];
        snprintf(verify, sizeof(verify), "G: %d", g_int);
        GFX_DrawString(2, 22, "Bias saved", GFX_COLOR_WHITE);
        GFX_DrawString(2, 32, verify, GFX_COLOR_WHITE);
        GFX_DrawString(2, 12, "Calib Complete", GFX_COLOR_WHITE);
        stableCount = 0;
        collectCount = 0;
        accelCalibState = CALIB_DONE;
    }

    // 每帧采集数据（不变）
    if (accelCalibStep < 6 && accelCalibState == CALIB_ACCEL_COLLECTING) {
        int16_t ax = (int16_t)((mpuDataBuffer[0] << 8) | mpuDataBuffer[1]);
        int16_t ay = (int16_t)((mpuDataBuffer[2] << 8) | mpuDataBuffer[3]);
        int16_t az = (int16_t)((mpuDataBuffer[4] << 8) | mpuDataBuffer[5]);
        accelData[accelCalibStep][0] += ax;
        accelData[accelCalibStep][1] += ay;
        accelData[accelCalibStep][2] += az;
        collectCount++;
        if (collectCount >= 100) { // 1s at 100Hz
            for (int j = 0; j < 3; j++) accelData[accelCalibStep][j] /= collectCount;
            accelCalibStep++;
            accelCalibState = CALIB_WAIT_BUTTON;
            collectCount = 0;
        }
    } else if (accelCalibState == CALIB_STABILIZING) {
        stableCount++;
        if (stableCount >= 300) { // 3s at 100Hz
            accelCalibState = CALIB_ACCEL_COLLECTING;
            stableCount = 0;
        }
    }
}

// 陀螺仪校准界面
static void UI_Gyro_Calib_Draw(UI_Widget* widget) {
    static uint8_t gyroCalibDone = 0;
    GFX_DrawString(2, 2, "Gyro Calib", GFX_COLOR_WHITE);
    if (!gyroCalibDone) {
        GFX_DrawString(2, 12, "Press key to start", GFX_COLOR_WHITE);
        if (gyroCalibState == CALIB_STABILIZING) {
            GFX_DrawString(2, 22, "Stabilizing...", GFX_COLOR_WHITE);
        } else if (gyroCalibState == CALIB_ACCEL_COLLECTING) {
            GFX_DrawString(2, 22, "Collecting...", GFX_COLOR_WHITE);
        }
    } else {
        GFX_DrawString(2, 12, "Bias saved", GFX_COLOR_WHITE);
        GFX_DrawString(2, 12, "Calib Complete", GFX_COLOR_WHITE);
        gyroCalibState = CALIB_DONE;
        stableCount = 0;
    }

    // 稳定等待
    if (gyroCalibState == CALIB_STABILIZING) {
        stableCount++;
        if (stableCount >= 300) { // 3s at 100Hz
            gyroCalibState = CALIB_ACCEL_COLLECTING;
            stableCount = 0;
        }
    }

    // 采集数据
    if (gyroCalibState == CALIB_ACCEL_COLLECTING) {
        int16_t gx = (int16_t)((mpuDataBuffer[8] << 8) | mpuDataBuffer[9]);
        int16_t gy = (int16_t)((mpuDataBuffer[10] << 8) | mpuDataBuffer[11]);
        int16_t gz = (int16_t)((mpuDataBuffer[12] << 8) | mpuDataBuffer[13]);
        gyroData[0] += gx;
        gyroData[1] += gy;
        gyroData[2] += gz;
        gyroSamples++;
        if (gyroSamples >= 300) { // 3s
            for (int j = 0; j < 3; j++) imuCalibData.gyro_bias[j] = gyroData[j] / gyroSamples;
            gyroCalibDone = 1;
        }
    }
}

// 磁力计校准界面
static void UI_Mag_Calib_Draw(UI_Widget* widget) {
    static uint8_t magCalibDone = 0;
    GFX_DrawString(2, 2, "Mag Calib", GFX_COLOR_WHITE);
    if (!magCalibDone) {
        GFX_DrawString(2, 12, "Press key to start", GFX_COLOR_WHITE);
        if (magCalibState == CALIB_STABILIZING) {
            GFX_DrawString(2, 22, "Stabilizing...", GFX_COLOR_WHITE);
        } else if (magCalibState == CALIB_ACCEL_COLLECTING) {
            GFX_DrawString(2, 22, "Collecting...", GFX_COLOR_WHITE);
        }
    } else {
        GFX_DrawString(2, 12, "Calib Complete", GFX_COLOR_WHITE);
        GFX_DrawString(2, 22, "Hard/Soft saved", GFX_COLOR_WHITE);
        magCalibState = CALIB_DONE;
        stableCount = 0;
    }

    // 稳定等待
    if (magCalibState == CALIB_STABILIZING) {
        stableCount++;
        if (stableCount >= 300) { // 3s
            magCalibState = CALIB_ACCEL_COLLECTING;
            stableCount = 0;
        }
    }

    // 采集数据
    if (magCalibState == CALIB_ACCEL_COLLECTING) {
        int16_t mx = (int16_t)((magDataBuffer[0] << 8) | magDataBuffer[1]);
        int16_t my = (int16_t)((magDataBuffer[2] << 8) | magDataBuffer[3]);
        int16_t mz = (int16_t)((magDataBuffer[4] << 8) | magDataBuffer[5]);
        // 存储样本（假设 magData 是数组）
        if (magSamples < 1000) {
            magData[magSamples][0] = mx;
            magData[magSamples][1] = my;
            magData[magSamples][2] = mz;
            magSamples++;
        }
        if (magSamples >= 1000) { // 采集完成
            // 计算硬铁：平均
            float hard[3] = {0};
            for (int i = 0; i < 1000; i++) {
                hard[0] += magData[i][0];
                hard[1] += magData[i][1];
                hard[2] += magData[i][2];
            }
            hard[0] /= 1000.0f;
            hard[1] /= 1000.0f;
            hard[2] /= 1000.0f;
            for (int j = 0; j < 3; j++) imuCalibData.mag_hard[j] = hard[j];

            // 计算软铁：简化，假设单位矩阵（实际需最小二乘拟合椭圆）
            for (int i = 0; i < 9; i++) imuCalibData.mag_soft[i][0] = (i % 4 == 0) ? 1.0f : 0.0f; // 单位矩阵

            magCalibDone = 1;
        }
    }
}


void UI_RenderFrame(void) {
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
}

# endif

void System_Update_Task() {
    Key_Toggle_Handler();
    LED_Toggle_Handler();

    Battery_Measure_Step();
    Read_IMU_All();
    Read_BMP_All();
}


void Start() {
    lastTime = sysTick;
    lastUpdateTick = sysTick;
    for (;;) {
        System_Update_Task();

        frameStart = sysTick;
        logicCounter++;
        dt = (uint16_t)(frameStart - lastUpdateTick) / 1000.0f;
        lastUpdateTick = frameStart;

        Attitude_Update(dt, &q_bias); 
        Locate_Update(dt, imu_ekf.q_corr);

        Quaternion q_target_ctrl = {1.0f, 0.0f, 0.0f, 0.0f};
        Quaternion q_target = q_target_ctrl;
        Math3D_QuatConjugate(&q_target);
        Math3D_QuatMultiply(&q_target, (Quaternion*)&imu_ekf.q_corr);
        float angle = 2.0f * acosf(q_target.w);
        Vector3D axis;
        axis.x = q_target.x;
        axis.y = q_target.y;
        axis.z = q_target.z;                 
        Math3D_VectorNormalize(&axis);
        Math3D_VectorMultiplyScalar(&axis, angle);
        rateSetRoll  = PID_Update(&pidRoll, 0.0f, axis.x, dt);
        rateSetPitch = PID_Update(&pidPitch, 0.0f, axis.y, dt);
        rateSetYaw   = PID_Update(&pidYaw, 0.0f, axis.z, dt);
        
        float heigthOutput = PID_Update(&pidHeight, 20.0f, loc_ekf.h, dt);
        thrustOutput = baseThrottle + heigthOutput;

        // 2. 蓝牙命令处理
        Parse_Bluetooth_Command();

        // 3. 渲染与屏幕刷新
        # ifdef DISPLAY_ENABLE
        UI_RenderFrame();
        # endif // DISPLAY_ENABLE

        if (Key_PressConsume()) {
            char pwm_status[64];
            sprintf(pwm_status, "PWM: %d, %d, %d, %d\r\n", 
                    TIM3->CCR1, TIM3->CCR2, TIM3->CCR3, TIM3->CCR4);
            Write_USART1_Data(pwm_status, strlen(pwm_status));


            # ifdef DISPLAY_ENABLE
            UI_Logger_AddLine(&logWindow, (char*)pwm_status);

            if (currentState == STATE_PID) {
                Quaternion q_target = {1.0f, 0.0f, 0.0f, 0.0f}; // 理想水平
                Quaternion* q_current = (Quaternion*)&imu_ekf.q;
                Quaternion q_current_conj = *q_current;
                Math3D_QuatConjugate(&q_current_conj);
                Math3D_QuatMultiply(&q_target, &q_current_conj); // q_target = q_target * q_current_conj
                q_bias = q_target;
                Save_Bias_Quaternion_To_Flash(&q_bias);

                // Quaternion qc = *(Quaternion*)&imu_ekf.q; // qc: w,x,y,z
                // // 计算 yaw（以 rad）
                // float yaw = atan2f(2.0f * (qc.w * qc.z + qc.x * qc.y),
                //                 1.0f - 2.0f * (qc.y * qc.y + qc.z * qc.z));
                // // 构造仅含 yaw 的四元数 q_yaw = [cos(yaw/2), 0, 0, sin(yaw/2)]
                // Quaternion q_yaw;
                // float hy = 0.5f * yaw;
                // q_yaw.w = cosf(hy);
                // q_yaw.x = 0.0f;
                // q_yaw.y = 0.0f;
                // q_yaw.z = sinf(hy);
                // // q_bias = q_yaw * q_current_conj  (保持 q_corr = q_bias * q_est 语义)
                // Quaternion q_curr_conj = qc;
                // Math3D_QuatConjugate(&q_curr_conj);
                // Math3D_QuatMultiply(&q_yaw, &q_curr_conj); // q_yaw <- q_yaw * q_curr_conj
                // q_bias = q_yaw;
                // Save_Bias_Quaternion_To_Flash(&q_bias);
            }

            // currentState = STATE_CUBE;
            currentState = STATE_MPU;
            // currentState = STATE_PID;
            // if (currentState == STATE_NONE) {
            //     currentState = STATE_ACCEL_CALIBRATE;
            // } else if (currentState == STATE_ACCEL_CALIBRATE) {
            //     if (accelCalibState == CALIB_WAIT_BUTTON && accelCalibStep < 6) {
            //         accelCalibState = CALIB_STABILIZING;
            //     } else if (accelCalibState == CALIB_DONE) {
            //         currentState = STATE_GYRO_CALIBRATE; // 进入陀螺
            //     }
            // } else if (currentState == STATE_GYRO_CALIBRATE) {
            //     if (gyroCalibState == CALIB_WAIT_BUTTON) {
            //         gyroCalibState = CALIB_STABILIZING;
            //     } else if (gyroCalibState == CALIB_DONE) {
            //         currentState = STATE_MAG_CALIBRATE;
            //     }
            // } else if (currentState == STATE_MAG_CALIBRATE) {
            //     if (magCalibState == CALIB_WAIT_BUTTON) {
            //         magCalibState = CALIB_STABILIZING;
            //     } else if (magCalibState == CALIB_DONE) {
            //         currentState = STATE_PID; // 回主界面
            //     }
            // }

            // # ifdef DISPLAY_ENABLE
            // char gravity[64];
            // // 打印重力加速度的模
            // {
            //     float g = sqrtf(gravity_est[0]*gravity_est[0] + gravity_est[1]*gravity_est[1] + gravity_est[2]*gravity_est[2]) / 9.81f;
            //     int mg = (int)(g * 1000.0f + 0.5f); // milli-g, rounded
            //     sprintf(gravity, "Gravity Init: %d mg", mg);
            // }
            // UI_Logger_AddLine(&logWindow, gravity);
            // # endif

            # endif // DISPLAY_ENABLE

            if (pwr_state == PWR_STATE_DISABLE) {
                Write_USART1_Data("PWR: ", Battery_GetPercentage());
                # ifdef DISPLAY_ENABLE
                char pwr_status[64];
                sprintf(pwr_status, "PWR, %d%%", Battery_GetPercentage());
                UI_Logger_AddLine(&logWindow, pwr_status);
                # endif
                pwr_state = PWR_STATE_PREPARE;
            }
        }

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


void Save_Bias_Quaternion_To_Flash(Quaternion* q) {
    // 擦除和写入4*float
    FLASH_Unlock();
    uint32_t sector = FLASH_Sector_5;
    FLASH_EraseSector(sector, VoltageRange_3);

    FLASH_ProgramWord(FLASH_BIAS_ADDR + 0 * 4, *((uint32_t*)&marker));

    FLASH_ProgramWord(FLASH_BIAS_ADDR + 1 * 4, *((uint32_t*)&q->w));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 2 * 4, *((uint32_t*)&q->x));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 3 * 4, *((uint32_t*)&q->y));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 4 * 4, *((uint32_t*)&q->z));

    FLASH_Lock();
}

void Load_Bias_Quaternion_From_Flash(Quaternion* q) {
    uint32_t* udata = (uint32_t*)FLASH_BIAS_ADDR;
    if (udata[0] == marker) {
        uint32_t wb = udata[1], xb = udata[2], yb = udata[3], zb = udata[4];
        memcpy(&q->w, &wb, sizeof(q->w));
        memcpy(&q->x, &xb, sizeof(q->x));
        memcpy(&q->y, &yb, sizeof(q->y));
        memcpy(&q->z, &zb, sizeof(q->z));
        float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
        if (!isfinite(norm) || norm < 0.5f || norm > 2.0f) {
            q->w = 1.0f; q->x = q->y = q->z = 0.0f;
        }
    }
    char buf[64];
    sprintf(buf, "Flash marker %d", (int)udata[0]);
    # ifdef DISPLAY_ENABLE
    UI_Logger_AddLine(&logWindow, buf);
    # endif // DISPLAY_ENABLE
}



// 建立双环控制（必须）√
// 为什么：内环角速率控制更快、更稳定；外环姿态控制生成角速设定。
// 改动点：新增高频定时中断（1kHz）做内环 Rate PID，主循环（100Hz）做姿态外环 P/PI 输出 rate_setpoint。
// 文件/位置建议：新增 control.c/timerISR 或在现有 systick handler 中加入定时器回调；把 Attitude->rate conversion 放到 Start() 的姿态处理处。
// 测试：在地面固定机臂 -> 给 step pitch setpoint，观察 gyro rate 跟随与电机响应。

// 确保 PID 保持状态并实现 anti-windup（必须）√
// 为什么：当前没有积分保持会导致I项失效；防止饱和时积分发散。
// 改动点：PID 结构加入 integrator、integrator_limit、last_error、D滤波；PID_Update 在输出饱和时停止/反向积分。
// 文件：pid.c / usystem.c PID_Init 初始化限幅。
// 测试：长时间施加偏差，观察 I 不发散且恢复正常。

// 用陀螺仪做内环反馈并对 gyro 做小 LPF（重要）
// 为什么：D项直接用 noisy gyro 会放大噪声；内环直接反馈角速。
// 改动点：对 gyro 做一阶 IIR（α~0.3-0.6），D 在差分前加小滤波或用 PT1。
// 测试：在地面抖动/敲击机臂，观测内环抖动程度。

// 限幅与混控饱和处理（必须）
// 为什么：防止单电机饱和导致失控，保持相对差值。
// 改动点：做 motor mixing 后检测 max/min，超出按比例缩放或优先保留 throttle（策略可选）。
// 代码片段：见下方“混控缩放”示例。
// 测试：全油门+roll大偏差，检查缩放是否合理且四电机未溢出。

// 时间步 dt 严格一致（必须）
// 为什么：PID、积分均依赖准确 dt。
// 改动点：内环用定时器的固定 dt；外环用主循环 dt。不要把 sysTick 差值作为混合 dt。
// 测试：打印每次 PID 调用的 dt 是否恒定。

// 输出平滑与低通（重要）
// 为什么：PWM（ESC）对高频指令无法跟随，直接传高频会振动。
// 改动点：对最终 motor command 做小一阶滤波（例如 α=0.2）。
// 测试：step 输入并观察 PWM 曲线是否平滑。

// 安全与状态机（必须）
// 为什么：断电、未解锁时防止电机转动；需要 ARM/DISARM。
// 改动点：引入 arming flag，按键/遥控安全逻辑，在未ARM时强制 PWM=MIN、PID 不积分。
// 测试：上电、未 ARM，推油门无反应；ARM 后解除限制并能控制。

// 建议：先调内环 Rate P（增益到稳定点），再调 Rate D，最后外环姿态 P、小 I。记录 step response。
// 提示值（起点，仅供调试）：
// Rate P: 0.1–1.0 (以角速度单位)
// Rate D: 0.005–0.05
// Attitude P: 4–8（你当前4可保留做逐步调整）

// telemetry/logging（重要）
// 为什么：没有日志难以定位振动点/不稳定频率。
// 改动点：在内环定时 ISR 中每间隔 N 次推送 key telemetry（gyro, rate_setpoint, motor[]）到串口或环形buffer。

// ESC/Throttle 线性化 与 电池电压补偿（可选）
// 为什么：电压下降时推力变化，需补偿。
// 改动点：在 PWM_Map_Percent 前乘电压系数。