#ifndef _ADC_H
#define _ADC_H

#include "main.h"

typedef struct
{
	uint16_t adcValues[2];
	uint16_t adcAverage[2];
	float temp1;
	float temp2;
}ADC_Value_t;

extern ADC_Value_t adc_value;

void TaskAdc(void *pvParameters);

#endif