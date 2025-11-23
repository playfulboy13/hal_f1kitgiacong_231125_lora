#include "7seg.h"

HC595_Handle_t hc595_handle;

static const uint8_t ma7doan[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
static uint8_t led_7dq[]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

void HC595_Init(void)
{
	hc595_handle.tt8led=0;
	hc595_handle.led_data=0x00;
	hc595_handle.display_mode=0;
}

void xuat_1_byte(uint8_t x)
{
	for(uint8_t i=0;i<8;i++)
	{
		(x&0x80)?(DS(1)):(DS(0));
		XUNG_DICH()
		x<<=1;
	}
}

void xuat_8led_7doan(uint8_t cot_hthi,uint8_t so_hthi)
{
	xuat_1_byte(hc595_handle.led_data);
	xuat_1_byte((uint8_t)~(1<<cot_hthi));
	xuat_1_byte(so_hthi);
	XUNG_CHOT()
}

void giai_ma_quet(void)
{
	led_7dq[0]=ma7doan[rtc_time.hours/10];
	if(rtc_time.bit_dao)
	{
		led_7dq[1]=ma7doan[rtc_time.hours%10];
	}
	else
	{
		led_7dq[1]=ma7doan[rtc_time.hours%10]&0x7f;
	}
	led_7dq[2]=ma7doan[rtc_time.minutes/10];
	led_7dq[3]=ma7doan[rtc_time.minutes%10];
	led_7dq[4]=ma7doan[rtc_time.seconds/10];
	led_7dq[5]=ma7doan[rtc_time.seconds%10];
	led_7dq[6]=0xff;
	led_7dq[7]=0xff;
}

void giai_ma_quet_2(void)
{
	uint8_t rssi2=abs(rssi);
	led_7dq[0]=ma7doan[counter/100%10];
	led_7dq[1]=ma7doan[counter/10%10];
	led_7dq[2]=ma7doan[counter%10];
	led_7dq[3]=0xff;
	led_7dq[4]=0xbf;
	led_7dq[5]=ma7doan[rssi2/100%10];
	led_7dq[6]=ma7doan[rssi2/10%10];
	led_7dq[7]=ma7doan[rssi2%10];
}

void TaskHienThi(void *pvParameters)
{
	HC595_Init();
	TickType_t lastTime=xTaskGetTickCount();
	while(1)
	{
		switch(hc595_handle.display_mode)
		{
			case 0: 
			{
				giai_ma_quet();
				break;
			}
			case 1: 
			{
				giai_ma_quet_2();
				break;
			}
			default: break;
		}
		xuat_8led_7doan(hc595_handle.tt8led,led_7dq[hc595_handle.tt8led]);
		hc595_handle.tt8led=(hc595_handle.tt8led+1)%8;
		vTaskDelayUntil(&lastTime,pdMS_TO_TICKS(5));
	}
}

