#ifndef __RADAR_H
#define __RADAR_H

#include "usart.h"
#include "map.h"
#include <stdbool.h>

#define RADAR_UART_LEFT (&huart8)
#define RADAR_UART_RIGHT (&huart4)

#define RADAR_DETECT_MAX_DIST 80

#define RADAR_FRAME_HEADER 0x6E
#define RADAR_FRAME_TAIL 0x62

#define RADAR_RX_BUF_SIZE 64

typedef struct
{
    bool has_target;   
    uint16_t distance;
} Radar_Data_t;

void Radar_Start(void);
void Radar_Stop(void);

void Radar_StopScanning(void);
void Radar_Update(void);

Direction_t Radar_GetAvoidanceDirection(void);
void Radar_DebugPrint(void);

#endif
