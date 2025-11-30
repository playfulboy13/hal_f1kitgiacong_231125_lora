#ifndef _USART3_H
#define _USART3_H

#include "main.h"

typedef struct
{
	SemaphoreHandle_t usart3_mutex;
}USART_Handle_t;

extern USART_Handle_t usart_handle;

void TaskSendData(void *pvParameters);

#endif
