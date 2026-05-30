#ifndef __LORA_TX_H
#define __LORA_TX_H

#include "usart.h"
#include "stdbool.h"

#define LORA_TARGET_ADDR_H 0x00
#define LORA_TARGET_ADDR_L 0x02
#define LORA_CHANNEL 0x14 //20 
#define LORA_UART (&huart5)


#define TEAM_NUMBER "6"
#define TEAM_NAME "404 not found"

void LoRa_Init(void);
void LoRa_SendTaskData_NonBlocking(uint32_t start_tick);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif

