#ifndef _USER_TASK_H
#define _USER_TASK_H
#include "main.h"

typedef struct
{
	uint8_t count;
}Task_Handle_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	
	uint8_t bit_dao;
}RTC_Time_t;

extern RTC_Time_t rtc_time;

extern int g_count;
extern float g_temp;
extern int rssi;

extern Task_Handle_t task_handle;

extern SemaphoreHandle_t lora_semaphore;

extern uint8_t counter;

void lora_init_config(void);

void Task1(void *pvParameters);
void TaskRTC(void *pvParameters);
void TaskLed(void *pvParameters);
void TaskButton(void *pvParameters);

void TaskSendLora(void *pvParameters);


void TaskMaster(void *pvParameters);


void TaskNode(void *pvParameters);

#endif
