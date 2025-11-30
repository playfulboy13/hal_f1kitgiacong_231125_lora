#include "usart3.h"

USART_Handle_t usart_handle;

// CRC8 co b?n
static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for(uint16_t i=0;i<len;i++)
    {
        crc ^= data[i];
        for(uint8_t j=0;j<8;j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

void TaskSendData(void *pvParameters)
{
    if(usart_handle.usart3_mutex == NULL)
        usart_handle.usart3_mutex = xSemaphoreCreateMutex();

    uint8_t buffer[514]; // STX + LEN + DATA(512) + CRC + ETX

    while(1)
    {
        if(xSemaphoreTake(usart_handle.usart3_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            uint16_t len = strlen(debug);
            if(len > 512) len = 512;

            uint16_t idx = 0;
            buffer[idx++] = 0x02;         // STX
            buffer[idx++] = (uint8_t)len; // LEN
            memcpy(&buffer[idx], debug, len);
            idx += len;
            buffer[idx++] = crc8((uint8_t*)debug, len); // CRC8
            buffer[idx++] = 0x03;         // ETX

            HAL_UART_Transmit_DMA(&huart3, buffer, idx);

            xSemaphoreGive(usart_handle.usart3_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}