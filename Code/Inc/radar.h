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
Direction_t Radar_LEFT_GetAvoidanceDirection(void);
void Radar_DebugPrint(void);

/* --- Data accessors for TFT display --- */
uint16_t Radar_GetLeftDistance(void);
uint16_t Radar_GetRightDistance(void);
bool Radar_GetLeftHasTarget(void);
bool Radar_GetRightHasTarget(void);
uint16_t Radar_GetLeftVotes(void);
uint16_t Radar_GetRightVotes(void);
bool Radar_IsScanning(void);

#endif
