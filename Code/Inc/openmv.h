#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"

typedef enum
{
    OpenMV_FLAG_NORMAL = 0x00,
    OpenMV_FLAG_CORNER_LEFT = 0x01,
    OpenMV_FLAG_CORNER_RIGHT = 0x02,
    OpenMV_FLAG_LOST = 0xFF
} OpenMV_Flag_t;
typedef struct
{
    int16_t error;      // 像素偏差值 (-160 到 160)
    OpenMV_Flag_t flag; 
    uint8_t is_updated; 
    uint32_t last_time;
} OpenMV_Data_t;


void OpenMV_Init(void);
OpenMV_Data_t *OpenMV_GetData(void);
void OpenMV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void OpenMV_ErrorCallback(UART_HandleTypeDef *huart);

#endif 
