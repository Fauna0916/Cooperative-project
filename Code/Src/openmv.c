#include "openmv.h"
#include "usart.h" 
#include <string.h>
#include <stdio.h>

#define OPENMV_UART huart3
#define OPENMV_RX_SIZE 64


ALIGN_32BYTES(uint8_t openmv_rx_buf[OPENMV_RX_SIZE])
__attribute__((section(".ARM.__at_0x24000100")));

static OpenMV_Data_t openmv_data = {0, FLAG_NORMAL, 0, 0};

void OpenMV_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_rx_buf, OPENMV_RX_SIZE);
    __HAL_DMA_DISABLE_IT(OPENMV_UART.hdmarx, DMA_IT_HT);
}

static void Parse_OpenMV(uint8_t *data, uint16_t len)
{
    // 1. 寻找 0xAA 帧头
    for (uint16_t i = 0; i <= (len - 5); i++)
    {
        if (data[i] == 0xAA)
        {
            // 2. 校验和验证
            uint8_t sum = data[i] + data[i + 1] + data[i + 2] + data[i + 3];
            if (sum == data[i + 4])
            {
                // 3. 位运算拼接
                // 小端序解析: data[i+1] 是低位, data[i+2] 是高位 (取决于OpenMV打包方式)
                // 如果 OpenMV 用 '<h', 则 i+1 是低, i+2 是高
                int16_t temp_err = (int16_t)(data[i + 2] << 8 | data[i + 1]);
                uint8_t temp_flg = data[i + 3];

                openmv_data.error = temp_err;
                openmv_data.flag = (OpenMV_Flag_t)temp_flg;
                openmv_data.is_updated = 1;
                openmv_data.last_time = HAL_GetTick();
                return; 
            }
        }
    }
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

OpenMV_Data_t *OpenMV_GetData(void)
{
    return &openmv_data;
}