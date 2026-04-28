#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"
#include "stdbool.h"

typedef enum
{
    OpenMV_FLAG_NORMAL = 0x00,
    OpenMV_FLAG_JUNC = 0x10,
    OpenMV_FLAG_LOST = 0xFF
} OpenMV_Flag_t;

typedef enum {
    Direction_FORWARD,
    Direction_LEFT,
    Direction_RIGHT,
    Direction_NORMAL,
} OpenMV_Possible_Direction_t;
typedef struct
{
    uint8_t flag;  // 当前的状态标志
    int16_t err_f; // 前方误差
    int16_t err_l; // 左转误差
    int16_t err_r; // 右转误差
    
    bool is_updated; 
    uint32_t last_time;
} OpenMV_Data_t;


void OpenMV_Init(void);
OpenMV_Data_t *OpenMV_GetData(void);
void OpenMV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void OpenMV_ErrorCallback(UART_HandleTypeDef *huart);

#endif 
