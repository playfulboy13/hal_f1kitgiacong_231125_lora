/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "user_task.h"

#include "adc.h"
#include "7seg.h"
#include "math.h"
#include "usart3.h"

extern SPI_HandleTypeDef hspi1;


extern uint8_t lora_flag;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern SemaphoreHandle_t button1_semaphore;


extern uint8_t button1_flag;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DS_Pin GPIO_PIN_12
#define DS_GPIO_Port GPIOB
#define SH_CP_Pin GPIO_PIN_13
#define SH_CP_GPIO_Port GPIOB
#define ST_CP_Pin GPIO_PIN_14
#define ST_CP_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_8
#define DIO0_GPIO_Port GPIOA
#define DIO0_EXTI_IRQn EXTI9_5_IRQn
#define RESET_Pin GPIO_PIN_11
#define RESET_GPIO_Port GPIOA
#define BT1_Pin GPIO_PIN_3
#define BT1_GPIO_Port GPIOB
#define BT1_EXTI_IRQn EXTI3_IRQn
#define BT2_Pin GPIO_PIN_4
#define BT2_GPIO_Port GPIOB
#define BT2_EXTI_IRQn EXTI4_IRQn
#define BT3_Pin GPIO_PIN_5
#define BT3_GPIO_Port GPIOB
#define BT3_EXTI_IRQn EXTI9_5_IRQn
#define NSS_Pin GPIO_PIN_9
#define NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
