#ifndef LORA_TX_H
#define LORA_TX_H

#include "main.h"
#include <stdint.h>


typedef struct
{
    UART_HandleTypeDef *huart_lora;
    UART_HandleTypeDef *huart_debug;

    uint16_t target_addr;
    uint8_t channel;

    uint32_t fake_start_hour;
    uint32_t fake_start_min;
    uint32_t fake_start_sec;

    uint8_t team_id;
    const char *name;

    uint32_t start_tick;
} LoRaTx_t;

void LoRaTx_Init(LoRaTx_t *lora,
                 UART_HandleTypeDef *huart_lora,
                 UART_HandleTypeDef *huart_debug,
                 uint16_t target_addr,
                 uint8_t channel,
                 uint8_t team_id,
                 const char *name,
                 uint32_t fake_start_hour,
                 uint32_t fake_start_min,
                 uint32_t fake_start_sec);

void LoRaTx_Start(LoRaTx_t *lora);

HAL_StatusTypeDef LoRaTx_SendTask2(LoRaTx_t *lora);

#endif
