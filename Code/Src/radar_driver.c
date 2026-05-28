#include "radar_driver.h"
#include "usart.h"
#include <string.h>

#define RADAR_FRAME_HEADER 0x6E // 固定的帧头
#define RADAR_FRAME_TAIL 0x62   // 固定的帧尾

static void RadarDriver_ParseFrame(RadarDriver_t *radar)
{
    if (radar->frame[0] != 0x6E || radar->frame[4] != 0x62)
    {
        radar->frame_ok = 0;
        return;
    }

    radar->target_state = radar->frame[1];

    // HLK-LD2410S uses little-endian format
    radar->distance_cm = ((uint16_t)radar->frame[3] << 8) | radar->frame[2];

    radar->frame_ok = 1;
    radar->last_update_tick = HAL_GetTick();
}

void RadarDriver_Init(RadarDriver_t *radar, UART_HandleTypeDef *huart)
{
    radar->huart = huart;
    radar->rx_byte = 0;
    memset((void *)radar->frame, 0, sizeof(radar->frame));
    radar->idx = 0;
    radar->target_state = 0;
    radar->distance_cm = 0;
    radar->frame_ok = 0;
    radar->last_update_tick = 0;
}

void RadarDriver_StartIT(RadarDriver_t *radar)
{
    __HAL_UART_CLEAR_OREFLAG(radar->huart);
    HAL_UART_Receive_IT(radar->huart, (uint8_t *)&radar->rx_byte, 1);
}

void RadarDriver_RxCpltCallback(RadarDriver_t *radar, UART_HandleTypeDef *huart)
{
    if (huart->Instance != radar->huart->Instance)
        return;

    uint8_t b = radar->rx_byte;

    // Wait for frame header 0x6E
    if (radar->idx == 0)
    {
        if (b == 0x6E)
        {
            radar->frame[radar->idx++] = b;
        }
    }
    else
    {
        radar->frame[radar->idx++] = b;

        if (radar->idx >= 5)
        {
            if (radar->frame[4] == 0x62)
            {
                RadarDriver_ParseFrame(radar);
            }
            else
            {
                radar->frame_ok = 0;
            }

            radar->idx = 0;
        }
    }

    HAL_UART_Receive_IT(radar->huart, (uint8_t *)&radar->rx_byte, 1);
}

uint8_t RadarDriver_IsValid(RadarDriver_t *radar, uint32_t timeout_ms)
{
    return (HAL_GetTick() - radar->last_update_tick) < timeout_ms;
}
