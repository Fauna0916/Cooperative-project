#include "openmv.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define OPENMV_UART huart3
#define OPENMV_RX_SIZE 64

ALIGN_32BYTES(uint8_t openmv_rx_buf[OPENMV_RX_SIZE])
__attribute__((section(".ARM.__at_0x30000000")));

uint8_t openmv_work_buf[OPENMV_RX_SIZE];

static OpenMV_Data_t openmv_data = {0, OpenMV_FLAG_NORMAL, 0, 0};

void OpenMV_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_rx_buf, OPENMV_RX_SIZE);
    __HAL_DMA_DISABLE_IT(OPENMV_UART.hdmarx, DMA_IT_HT);
}

static void Parse_OpenMV(uint8_t *data, uint16_t len)
{
    if (len < 5)
        return;

    // Loop through the buffer to find the header
    for (uint16_t i = 0; i <= (len - 5); i++)
    {
        if (data[i] == 0xAA)
        {
            // Calculate checksum (uint8_t handles the 8-bit wrap-around/mask automatically)
            uint8_t sum = data[i] + data[i + 1] + data[i + 2] + data[i + 3];

            // Check against the received checksum byte
            if (sum == data[i + 4])
            {
                // data[i+1] is Low Byte, data[i+2] is High Byte
                int16_t temp_err = (int16_t)((uint16_t)data[i + 2] << 8 | data[i + 1]);
                uint8_t temp_flg = data[i + 3];

                // Store data
                openmv_data.error = temp_err;
                openmv_data.flag = (OpenMV_Flag_t)temp_flg;
                openmv_data.is_updated = 1;
                openmv_data.last_time = HAL_GetTick();

                return;
            }
        }
    }
}

OpenMV_Data_t *OpenMV_GetData(void)
{
    return &openmv_data;
}

void OpenMV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == OPENMV_UART.Instance)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)openmv_rx_buf, OPENMV_RX_SIZE);

        Parse_OpenMV(openmv_rx_buf, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_rx_buf, OPENMV_RX_SIZE);
        __HAL_DMA_DISABLE_IT(OPENMV_UART.hdmarx, DMA_IT_HT);
    }
}

void OpenMV_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == OPENMV_UART.Instance)
    {
        uint32_t temp;
        // Read SR/ISR and then DR/RDR to clear ORE
        temp = huart->Instance->ISR;
        temp = huart->Instance->RDR;
        (void)temp;
        // Restart DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_rx_buf, OPENMV_RX_SIZE);
    }
}
