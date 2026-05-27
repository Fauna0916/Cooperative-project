#include "radar_driver.h"
#include "usart.h"
#include <string.h>

#define RADAR_FRAME_HEADER 0x6E // 固定的帧头
#define RADAR_FRAME_TAIL 0x62   // 固定的帧尾

static void RadarDriver_ParseFrame(RadarDriver_t *radar)
{
    // 这里按你们原来 5 字节协议解析
    // 假设:
    // frame[0] = 帧头
    // frame[1] = target_state
    // frame[2] = distance high
    // frame[3] = distance low
    // frame[4] = 帧尾 / 校验
    //
    // 如果你们实际协议不是这样，再换成你原来的解析逻辑

    radar->target_state = radar->frame[1];
    radar->distance_cm = ((uint16_t)radar->frame[2] << 8) | radar->frame[3];
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

    radar->frame[radar->idx++] = radar->rx_byte;

    if (radar->idx >= 5)
    {
        RadarDriver_ParseFrame(radar);
        radar->idx = 0;
    }

    HAL_UART_Receive_IT(radar->huart, (uint8_t *)&radar->rx_byte, 1);
}

uint8_t RadarDriver_IsValid(RadarDriver_t *radar, uint32_t timeout_ms)
{
    return (HAL_GetTick() - radar->last_update_tick) < timeout_ms;
}
