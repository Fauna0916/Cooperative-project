#include "lora_tx.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

LoRaTx_t lora_tx;

void LoRaTx_Init(LoRaTx_t *lora,
                 UART_HandleTypeDef *huart_lora,
                 UART_HandleTypeDef *huart_debug,
                 uint16_t target_addr,
                 uint8_t channel,
                 uint8_t team_id,
                 const char *name,
                 uint32_t fake_start_hour,
                 uint32_t fake_start_min,
                 uint32_t fake_start_sec)
{
    lora->huart_lora = huart_lora;
    lora->huart_debug = huart_debug;

    lora->target_addr = target_addr;
    lora->channel = channel;

    lora->team_id = team_id;
    lora->name = name;

    lora->fake_start_hour = fake_start_hour;
    lora->fake_start_min = fake_start_min;
    lora->fake_start_sec = fake_start_sec;

    lora->start_tick = 0;
}

void LoRaTx_Start(LoRaTx_t *lora)
{
    lora->start_tick = HAL_GetTick();
}

void LoRa_Init(void)
{
    LoRaTx_Init(&lora_tx,
                &huart5,
                &huart2,
                0x0002,
                20,
                6,
                "Lora_test",
                12, 0, 0);

    LoRaTx_Start(&lora_tx);
}

HAL_StatusTypeDef LoRaTx_SendTask2(LoRaTx_t *lora)
{
    char msg[160];
    uint8_t tx_buf[200];
    uint16_t msg_len;

    uint32_t elapsed_ms = HAL_GetTick() - lora->start_tick;
    uint32_t total_sec = elapsed_ms / 1000U;

    uint32_t elapsed_min = total_sec / 60U;
    uint32_t elapsed_sec = total_sec % 60U;

    uint32_t current_total_sec = lora->fake_start_hour * 3600U
                               + lora->fake_start_min * 60U
                               + lora->fake_start_sec
                               + total_sec;

    uint32_t current_hour = (current_total_sec / 3600U) % 24U;
    uint32_t current_min  = (current_total_sec % 3600U) / 60U;
    uint32_t current_sec  = current_total_sec % 60U;

    snprintf(msg, sizeof(msg),
             "TIME=%02u:%02u:%02u,TEAM=%u,NAME=%s,ELAPSED=%02u:%02u",
             current_hour,
             current_min,
             current_sec,
             lora->team_id,
             lora->name,
             elapsed_min,
             elapsed_sec);

    msg_len = (uint16_t)strlen(msg);

    // Fixed mode header
    tx_buf[0] = (uint8_t)((lora->target_addr >> 8) & 0xFF);
    tx_buf[1] = (uint8_t)(lora->target_addr & 0xFF);
    tx_buf[2] = lora->channel;

    memcpy(&tx_buf[3], msg, msg_len);

    HAL_StatusTypeDef status = HAL_UART_Transmit(lora->huart_lora, tx_buf, msg_len + 3U, 100);

    if (lora->huart_debug != NULL)
    {
        HAL_UART_Transmit(lora->huart_debug, (uint8_t *)msg, msg_len, 100);
        HAL_UART_Transmit(lora->huart_debug, (uint8_t *)"\r\n", 2, 100);
    }

    return status;
}
