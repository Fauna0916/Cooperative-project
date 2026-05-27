#ifndef RADAR_DRIVER_H
#define RADAR_DRIVER_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

typedef struct
{
    UART_HandleTypeDef *huart;

    volatile uint8_t rx_byte;
    volatile uint8_t frame[5];
    volatile uint8_t idx;

    volatile uint8_t target_state;
    volatile uint16_t distance_cm;
    volatile uint8_t frame_ok;

    uint32_t last_update_tick;
} RadarDriver_t;

void RadarDriver_Init(RadarDriver_t *radar, UART_HandleTypeDef *huart);
void RadarDriver_StartIT(RadarDriver_t *radar);
void RadarDriver_RxCpltCallback(RadarDriver_t *radar, UART_HandleTypeDef *huart);

uint8_t RadarDriver_IsValid(RadarDriver_t *radar, uint32_t timeout_ms);

#endif
