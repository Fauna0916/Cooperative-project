#include "openmv.h"
#include "usart.h"
#include <string.h>

#define OPENMV_UART huart3
#define OPENMV_RX_SIZE 32

ALIGN_32BYTES(uint8_t openmv_rx_buf[OPENMV_RX_SIZE])
__attribute__((section(".ARM.__at_0x30000000")));

static OpenMV_Data_t openmv_data = {0};

void OpenMV_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_rx_buf, OPENMV_RX_SIZE);
    __HAL_DMA_DISABLE_IT(OPENMV_UART.hdmarx, DMA_IT_HT);
}

static void Parse_OpenMV(uint8_t *data, uint16_t len)
{
    // 新协议总长度为 9 字节
    if (len < 9)
        return;

    // 寻找包头 0xAA
    for (uint16_t i = 0; i <= (len - 9); i++)
    {
        if (data[i] == 0xAA)
        {
            // 校验和计算 (sum(byte 0 到 byte 7))
            uint8_t sum = 0;
            for (int j = 0; j < 8; j++)
            {
                sum += data[i + j];
            }

            // 对比第 9 个字节 (data[i+8])
            if (sum == data[i + 8])
            {
                // 解析标志位
                openmv_data.flag = data[i + 1];

                // 解析三个 int16_t 误差 (低位在前，高位在后)
                openmv_data.err_f = (int16_t)((uint16_t)data[i + 3] << 8 | data[i + 2]);
                openmv_data.err_l = (int16_t)((uint16_t)data[i + 5] << 8 | data[i + 4]);
                openmv_data.err_r = (int16_t)((uint16_t)data[i + 7] << 8 | data[i + 6]);

                openmv_data.is_updated = true;
                openmv_data.last_time = HAL_GetTick();

                return; // 解析成功，退出
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
