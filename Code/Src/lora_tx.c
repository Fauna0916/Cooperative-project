#include "lora_tx.h"
#include <stdio.h>
#include <string.h>

static uint8_t lora_tx_buffer[256];
static bool is_lora_busy = false;
static uint32_t last_trigger_tick = 0;

void LoRa_Init(void)
{
    is_lora_busy = false;
}

// TODO: USE RTC TIME
void LoRa_SendTaskData_NonBlocking(uint32_t start_tick)
{
    if (HAL_GetTick() - last_trigger_tick < 150)
    {
        return;
    }
    last_trigger_tick = HAL_GetTick();

    if (HAL_UART_GetState(LORA_UART) == HAL_UART_STATE_BUSY_TX || is_lora_busy)
    {
        return;
    }

    char payload[128];
    uint32_t current_tick = HAL_GetTick();
    uint32_t duration_s = (current_tick - start_tick) / 1000;
    uint16_t min = (uint16_t)(duration_s / 60);
    uint16_t sec = (uint16_t)(duration_s % 60);
    uint32_t sys_s = current_tick / 1000;

    // 2. 准备数据负载
    int payload_len = sprintf(payload, "Gp:%s,Name:%s,Time:%02u:%02u:%02u,Dur:%02d:%02d\r\n",
                              TEAM_NUMBER, TEAM_NAME,
                              (unsigned int)((sys_s / 3600) % 24),
                              (unsigned int)((sys_s / 60) % 60),
                              (unsigned int)(sys_s % 60),
                              min, sec);

    // 3. 组装固定模式帧头 [目标地址H, 目标地址L, 信道]
    lora_tx_buffer[0] = LORA_TARGET_ADDR_H;
    lora_tx_buffer[1] = LORA_TARGET_ADDR_L;
    lora_tx_buffer[2] = LORA_CHANNEL;

    // 4. 将负载拷贝到全局缓冲区
    memcpy(&lora_tx_buffer[3], payload, (size_t)payload_len);


    is_lora_busy = true;
    if (HAL_UART_Transmit_IT(LORA_UART, lora_tx_buffer, (uint16_t)(payload_len + 3)) != HAL_OK)
    {
        is_lora_busy = false; 
    }
}

void LoRa_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LORA_UART->Instance)
    {
        is_lora_busy = false;
    }
}