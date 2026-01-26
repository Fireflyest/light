# include "pwr.h"

static float divider_scale(void);
static void adc_enable_once(void);
static void adc_disable_once(void);

__IO uint16_t pwr_adc_raw = 0;
__IO uint16_t pwr_state = PWR_STATE_DISABLE;

void Init_PWR(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // Enable GPIOC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  // PC0 as analog input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

void PWR_Handle(void) {
    if (pwr_state == PWR_STATE_PREPARE) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        pwr_state = PWR_STATE_READ;
    } else if (pwr_state == PWR_STATE_READ) {
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles); // PC0 is ADC Channel 10
        ADC_SoftwareStartConv(ADC1);
        pwr_state = PWR_STATE_WAIT;
    } else if (pwr_state == PWR_STATE_DISABLE) {
        // pwr_state = PWR_STATE_PREPARE;
    }
}

uint8_t PWR_GetPercentage(void) {
    float batt_v = (pwr_adc_raw / (float)ADC_MAX) * VREF_V * divider_scale();
    uint32_t batt_mv = (uint32_t)(batt_v * 1000.0f + 0.5f);

    if (batt_mv >= BATTERY_FULL) return 100;
    if (batt_mv <= BATTERY_EMPTY) return 0;

    float pct = ((float)(batt_mv - BATTERY_EMPTY)) * 100.0f / (float)(BATTERY_FULL - BATTERY_EMPTY);
    return (uint8_t)(pct + 0.5f);
}

static float divider_scale(void) {
    return (R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;
}
