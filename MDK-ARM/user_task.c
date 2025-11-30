#include "user_task.h"

#include "LoRa.h"

#define FRAME_START   0xAA
#define FRAME_END     0x55

#define CMD_READ_REQ  0x00
#define CMD_READ_ACK  0x01


Task_Handle_t task_handle;
SemaphoreHandle_t lora_semaphore;

uint8_t relay_state;

char debug[256];

LoRa myLoRa;
int g_count = 0;
float g_temp = 0.0f;
int rssi = 0;

uint8_t counter;

RTC_Time_t rtc_time;

void lora_init_config(void)
{
		myLoRa = newLoRa();
    myLoRa.CS_port         = NSS_GPIO_Port;
    myLoRa.CS_pin          = NSS_Pin;
    myLoRa.reset_port      = RESET_GPIO_Port;
    myLoRa.reset_pin       = RESET_Pin;
    myLoRa.DIO0_port       = DIO0_GPIO_Port;
    myLoRa.DIO0_pin        = DIO0_Pin;
    myLoRa.hSPIx           = &hspi1;

    myLoRa.frequency             = 435;
    myLoRa.spredingFactor        = SF_7;
    myLoRa.bandWidth             = BW_125KHz;
    myLoRa.crcRate               = CR_4_5;
    myLoRa.power                 = POWER_20db;
    myLoRa.overCurrentProtection = 100;
    myLoRa.preamble              = 8;

    LoRa_init(&myLoRa);
}
/*RECEIVE PACKAGE*/
void Task1(void *pvParameters)
{
		lora_init_config();
    LoRa_startReceiving(&myLoRa);
    uint8_t rx_buf[32];
    uint8_t size = 0;
    char uart_buffer[256];

    while (1)
    {
        if (xSemaphoreTake(lora_semaphore,portMAX_DELAY)==pdTRUE)
        {
            lora_flag = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
            size = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));
					  LoRa_startReceiving(&myLoRa);
            rssi = LoRa_getRSSI(&myLoRa);
					
					   if (size >= 16)
            {
                // ==== 1. Ki?m tra Start và End ====
                if (rx_buf[0] == 0xAA && rx_buf[15] == 0x55)
                {
                    // ==== 2. Ki?m tra checksum t? byte 1 -> 13 ====
                    uint8_t checksum = 0;
                    for (int i = 1; i <= 13; i++) checksum ^= rx_buf[i];

                    if (checksum == rx_buf[14])
                    {
                        // ==== 3. Parse d? li?u ====
                        uint8_t node_id = rx_buf[1];
                       counter = rx_buf[2];

                        float temp = 0, humidity = 0;
                        memcpy(&temp, &rx_buf[3], 4);
                        memcpy(&humidity, &rx_buf[7], 4);

                        uint8_t d1 = rx_buf[11];
                        uint8_t d2 = rx_buf[12];
                        uint8_t d3 = rx_buf[13];

                        // ==== 4. In ra UART ====
                        sprintf(uart_buffer,
                                "\r\n[LoRa Packet]\r\n"
                                "Node ID: %d\r\n"
                                "Counter: %d\r\n"
                                "Temp    : %.2f\r\n"
                                "Humidity: %.2f\r\n"
                                "Extra   : %d %d %d\r\n"
                                "RSSI    : %d dBm\r\n",
                                node_id, counter, temp, humidity,
                                d1, d2, d3, rssi);

                        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer));
												HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin); // blink LED
                    }
                    else
                    {
                        HAL_UART_Transmit(&huart1,
                            (uint8_t*)"Checksum ERROR\r\n", 16, 100);
                    }
                }
                else
                {
                    HAL_UART_Transmit_DMA(&huart1,
                        (uint8_t*)"Frame ERROR\r\n", 13);
                }
            }
				}
				 vTaskDelay(pdMS_TO_TICKS(10)); // nhu?ng CPU
    }
}
/*SEND PACKAGE*/
void TaskSendLora(void *pvParameters)
{
	   lora_init_config();

    uint8_t tx_buffer[32];
    uint8_t node_id = 0xA1;  // Node ID
    float temp = 25.0f;
    float humidity = 60.0f;  // Gi? l?p d? ?m ban d?u

    while (1)
    {
        // Gi? l?p c?m bi?n
      

        humidity += ((rand() % 100) - 50) / 200.0f;
        if (humidity < 40.0f) humidity = 40.0f;
        if (humidity > 90.0f) humidity = 90.0f;

        // Ðóng gói
        tx_buffer[0] = 0xAA;         // Start
        tx_buffer[1] = 23;      // Node ID
        //tx_buffer[2] = 79;        // Counter
			
				memcpy(&tx_buffer[2],&counter,sizeof(uint8_t));

        memcpy(&tx_buffer[3], &temp, sizeof(float));       // Nhi?t d?
        memcpy(&tx_buffer[7], &humidity, sizeof(float));   // Ð? ?m
				

        tx_buffer[11] = 23;
        tx_buffer[12] = 56;
        tx_buffer[13] = 46;

        // Checksum XOR t? byte 1 ? 13
        uint8_t checksum = 0;
        for (int i = 1; i <= 13; i++)
            checksum ^= tx_buffer[i];
        tx_buffer[14] = checksum;

        tx_buffer[15] = 0x55;        // End

        // G?i qua LoRa
        if (LoRa_transmit(&myLoRa, tx_buffer, 16, 100) == 1) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

				(counter<100)?(counter++):(counter=0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskRTC(void *pvParameters)
{
	rtc_time.bit_dao=0;
	while(1)
	{
		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
		
		rtc_time.seconds=sTime.Seconds;
		rtc_time.minutes=sTime.Minutes;
		rtc_time.hours=sTime.Hours;
		rtc_time.date=sDate.Date;
		rtc_time.day=sDate.WeekDay;
		rtc_time.month=sDate.Month;
		rtc_time.year=sDate.Year;
		
		rtc_time.bit_dao=!rtc_time.bit_dao;
		
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

#define LED_NUM 4
const uint8_t led_bits[LED_NUM] = {LED1_BIT, LED2_BIT, LED3_BIT, LED4_BIT};

void TaskLed(void *pvParameters)
{
    while(1)
    {
        for(uint8_t i = 0; i < LED_NUM; i++)
        {
            if(relay_state & (1 << i))
                set_led_bit(led_bits[i], 1);  // B?t LED
            else
                set_led_bit(led_bits[i], 0);  // T?t LED
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void TaskButton(void *pvParameters)
{
	TickType_t LastTime=0;
	const TickType_t DelayDebounce=pdMS_TO_TICKS(300);
	while(1)
	{
		if(button1_flag)
		{
			TickType_t CurrentTime=xTaskGetTickCount();
			
			if(CurrentTime-LastTime>=DelayDebounce)
			{
				button1_flag=0;
				hc595_handle.display_mode++;
				if(hc595_handle.display_mode>1)
				{
					hc595_handle.display_mode=0;
				}
				LastTime=CurrentTime;
			}
		}
		 vTaskDelay(pdMS_TO_TICKS(10)); // nhu?ng CPU
	}
}


//////////////////////

uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void float_to_bytes(float f, uint8_t *buf)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = f;
    memcpy(buf, u.b, 4);
}

float bytes_to_float(uint8_t *buf)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    memcpy(u.b, buf, 4);
    return u.f;
}



#define MAX_RETRY       3
#define CMD_ACK         0xA1

void TaskMaster(void *pvParameters)
{
    lora_init_config();

    uint8_t rx_buf[64];
    uint8_t size;

    uint8_t node_id = 0x01;

    while (1)
    {
        bool success = false;

        for (int retry = 0; retry < MAX_RETRY; retry++)
        {
            // ==== ENABLE RX TRU?C KHI G?I REQUEST (GI?M M?T GÓI) ====
            LoRa_startReceiving(&myLoRa);
            vTaskDelay(pdMS_TO_TICKS(2));

            // ==== SEND REQUEST ====
            uint8_t request[4] = {FRAME_START, node_id, CMD_READ_REQ, FRAME_END};
            LoRa_transmit(&myLoRa, request, 4, 100);

            char log_tx[60];
            sprintf(log_tx, "MASTER: Request sent to NODE %02X (retry=%d)\r\n", node_id, retry);
            HAL_UART_Transmit(&huart1, (uint8_t*)log_tx, strlen(log_tx), 20);

            uint32_t t0 = xTaskGetTickCount();
            bool received = false;

            while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(300))
            {
                if (lora_flag)
                {
                    lora_flag = 0;
                    size = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));
									HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

                    // ==== CHECK DATA FRAME ====
                    if (size >= 40 &&
                        rx_buf[0] == FRAME_START &&
                        rx_buf[1] == node_id &&
                        rx_buf[size - 1] == FRAME_END)
                    {
                        uint8_t len = rx_buf[2];
                        if (len != 34) continue;

                        // CRC RECEIVED
                        uint16_t crc_recv =
                            ((uint16_t)rx_buf[3 + 34] << 8) |
                            (rx_buf[3 + 34 + 1]);

                        // CRC CALC
                        uint16_t crc_calc = crc16_ccitt(&rx_buf[1], 1 + 1 + 34);

                        if (crc_calc != crc_recv)
                        {
                            char msg[] = "MASTER: CRC ERROR!\r\n";
                            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 20);
                            continue;
                        }

                        // Decode float
                        float sensor[8];
                        int offset = 3;
                        for (int i = 0; i < 8; i++)
                        {
                            sensor[i] = bytes_to_float(&rx_buf[offset]);
                            offset += 4;
                        }

                        counter = rx_buf[offset++];
                        uint8_t custom_param = rx_buf[offset++];
                        rssi = LoRa_getRSSI(&myLoRa);

                        // DISPLAY
                       
                        sprintf(debug,
                            "MASTER (NODE=%02X OK, retry=%d):\r\n"
                            "S0=%.2f S1=%.2f S2=%.2f S3=%.2f\r\n"
                            "S4=%.2f S5=%.2f S6=%.2f S7=%.2f\r\n"
                            "CNT=%d PARAM=0x%02X RSSI=%d\r\n",
                            node_id, retry,
                            sensor[0], sensor[1], sensor[2], sensor[3],
                            sensor[4], sensor[5], sensor[6], sensor[7],
                            counter, custom_param, rssi);

                        HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 20);

                        received = true;
                        break;
                    }

                    // ==== CHECK ACK FRAME ====
                    if (size == 4 &&
                        rx_buf[0] == FRAME_START &&
                        rx_buf[1] == node_id &&
                        rx_buf[2] == CMD_ACK &&
                        rx_buf[3] == FRAME_END)
                    {
                        char ack_msg[40];
                        sprintf(ack_msg, "MASTER: ACK received from NODE %02X\r\n", node_id);
                        HAL_UART_Transmit(&huart1, (uint8_t*)ack_msg, strlen(ack_msg), 20);
                        continue;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }

            if (received)
            {
                success = true;
                break;
            }

            // ==== RETRY LOG ====
            char msg2[60];
            sprintf(msg2, "MASTER: Retry %d for NODE %02X...\r\n", retry+1, node_id);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), 20);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!success)
        {
            char err[80];
            sprintf(err, "MASTER: NO RESPONSE FROM NODE %02X! Will poll again...\r\n", node_id);
            HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), 20);
        }

        // ==== WAIT BEFORE NEXT POLLING ====
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}





#define CMD_RELAY_CTRL   0xB1
#define CMD_ACK          0xA1

void TaskNode(void *pvParameters)
{
    lora_init_config();

    uint8_t rx_buf[32];
    uint8_t size;

    uint8_t node_id = 0x01;

    float sensor[8] = {
        12.34, 56.78, 90.12, 34.56,
        78.90, 11.22, 33.44, 55.66
    };

    uint8_t custom_param = 0x55;

    LoRa_startReceiving(&myLoRa);

    while (1)
    {
        if (lora_flag)
        {
            lora_flag = 0;

            size = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));
            rssi = LoRa_getRSSI(&myLoRa);

            // Khung không h?p l?
            if (size < 4 || rx_buf[0] != FRAME_START || rx_buf[size-1] != FRAME_END)
                goto CONTINUE_RECEIVE;

            // ======================================
            //      1) X? LÝ L?NH ÐI?U KHI?N RELAY
            // ======================================
            if (size >= 7 &&
                rx_buf[1] == node_id &&
                rx_buf[2] == CMD_RELAY_CTRL)
            {
                relay_state = rx_buf[3];

                // tính CRC: node_id + CMD + relay_state
                uint16_t crc_recv = (rx_buf[4] << 8) | rx_buf[5];
                uint16_t crc_calc = crc16_ccitt(&rx_buf[1], 3);

                if (crc_recv == crc_calc)
                {
                    char debug[64];
                    sprintf(debug,
                            "NODE %02X RELAY CMD: state=0x%02X\r\n",
                            node_id, relay_state);
                    HAL_UART_Transmit(&huart1,
                                      (uint8_t*)debug, strlen(debug), 50);
                }
                else
                {
                    HAL_UART_Transmit(&huart1,
                        (uint8_t*)"NODE RELAY CMD CRC ERROR\r\n",
                        28, 50);
                }

                goto CONTINUE_RECEIVE;
            }

            // ======================================
            //      2) X? LÝ L?NH Ð?C SENSOR
            // ======================================
            if (size == 4 &&
                rx_buf[1] == node_id &&
                rx_buf[2] == CMD_READ_REQ)
            {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

                // C?p nh?t sensor
                sensor[0] = adc_value.temp1;
                sensor[1] = adc_value.temp2;

                uint8_t reply[64];
                uint8_t idx = 0;

                reply[idx++] = FRAME_START;
                reply[idx++] = node_id;
                reply[idx++] = 34; // LEN DATA

                for (int i = 0; i < 8; i++)
                {
                    float_to_bytes(sensor[i], &reply[idx]);
                    idx += 4;
                }

                reply[idx++] = counter++;
                reply[idx++] = custom_param;

                uint16_t crc = crc16_ccitt(&reply[1], 1 + 1 + 34);
                reply[idx++] = (crc >> 8) & 0xFF;
                reply[idx++] = crc & 0xFF;

                reply[idx++] = FRAME_END;

                // G?i DATA FRAME
                LoRa_transmit(&myLoRa, reply, idx, 100);

                // G?i ACK FRAME
                uint8_t ack[4] = {FRAME_START, node_id, CMD_ACK, FRAME_END};
                LoRa_transmit(&myLoRa, ack, 4, 50);

                LoRa_startReceiving(&myLoRa);
                vTaskDelay(pdMS_TO_TICKS(5));

                goto CONTINUE_RECEIVE;
            }

CONTINUE_RECEIVE:
            LoRa_startReceiving(&myLoRa);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}








