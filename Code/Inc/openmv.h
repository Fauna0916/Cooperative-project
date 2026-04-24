#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"

typedef enum
{
    OpenMV_FLAG_NORMAL = 0, // 正常巡线
    OpenMV_FLAG_CORNER = 1, // 发现90度直角或十字路口
    OpenMV_FLAG_LOST = 0xFF    // 丢线 (可能前方是障碍盒或虚线)
} OpenMV_Flag_t;

typedef struct
{
    int16_t error;      // 像素偏差值 (-160 到 160)
    OpenMV_Flag_t flag; // 路况状态
    uint8_t is_updated; // 更新标志 (1=有新数据，0=已处理)
    uint32_t last_time; // 上次接收到数据的时间戳 (用于掉线保护)
} OpenMV_Data_t;


void OpenMV_Init(void);
OpenMV_Data_t *OpenMV_GetData(void);
void OpenMV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void OpenMV_ErrorCallback(UART_HandleTypeDef *huart);

#endif 
