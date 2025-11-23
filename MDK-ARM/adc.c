#include "adc.h"
#include <math.h>

#define ADC_MAX      4095.0f
#define VREF         3.3f
#define NTC_R1       10000.0f  // Ði?n tr? phân áp
#define NTC_R0       10000.0f  // NTC 10k t?i 25°C
#define NTC_B        3950.0f   // H?ng s? Beta
#define T0_K         298.15f   // 25°C = 298.15K
#define MOVING_AVG_LEN 8       // S? m?u trung bình tru?t

ADC_Value_t adc_value;

// buffer cho trung bình tru?t
static uint16_t adcBuf0[MOVING_AVG_LEN] = {0};
static uint16_t adcBuf1[MOVING_AVG_LEN] = {0};
static uint8_t idx = 0;

float adc_to_temp(uint16_t adc)
{
    float v = (float)adc / ADC_MAX * VREF;
    float r = NTC_R1 * v / (VREF - v);
    float tempK = NTC_B / (logf(r / NTC_R0) + NTC_B / T0_K);
    return tempK - 273.15f; // °C
}

void TaskAdc(void *pvParameters)
{
    while(1)
    {
        // --- Luu ADC m?i vào buffer ---
        adcBuf0[idx] = adc_value.adcValues[0];
        adcBuf1[idx] = adc_value.adcValues[1];

        // --- Tính trung bình tru?t ---
        uint32_t sum0 = 0, sum1 = 0;
        for(uint8_t i=0; i<MOVING_AVG_LEN; i++)
        {
            sum0 += adcBuf0[i];
            sum1 += adcBuf1[i];
        }
        adc_value.adcAverage[0] = sum0 / MOVING_AVG_LEN;
        adc_value.adcAverage[1] = sum1 / MOVING_AVG_LEN;

        // --- Chuy?n sang d? C ---
        adc_value.temp1 = adc_to_temp(adc_value.adcAverage[0]);
        adc_value.temp2 = adc_to_temp(adc_value.adcAverage[1]);

        // --- tang index ---
        idx++;
        if(idx >= MOVING_AVG_LEN) idx = 0;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
