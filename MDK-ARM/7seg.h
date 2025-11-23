#ifndef _7SEG_H
#define _7SEG_H

#include "main.h"

#include "user_task.h"

#define DS(x) HAL_GPIO_WritePin(DS_GPIO_Port,DS_Pin,(x)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define SH_CP(x) HAL_GPIO_WritePin(SH_CP_GPIO_Port,SH_CP_Pin,(x)?(GPIO_PIN_SET):(GPIO_PIN_RESET))
#define ST_CP(x) HAL_GPIO_WritePin(ST_CP_GPIO_Port,ST_CP_Pin,(x)?(GPIO_PIN_SET):(GPIO_PIN_RESET))

#define XUNG_DICH() {SH_CP(1);SH_CP(0);}
#define XUNG_CHOT() {ST_CP(1);ST_CP(0);}

typedef struct
{
	uint8_t tt8led;
	uint8_t led_data;
	uint8_t display_mode;
}HC595_Handle_t;

extern HC595_Handle_t hc595_handle;

void HC595_Init(void);
void xuat_1_byte(uint8_t x);
void xuat_8led_7doan(uint8_t cot_hthi,uint8_t so_hthi);
void giai_ma_quet(void);
void giai_ma_quet_2(void);
void TaskHienThi(void *pvParameters);

#endif
