#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdbool.h>
#include "gpio.h"
#include "map.h"

typedef enum
{
    GraySensor_FLAG_NORMAL = 0x00,
    GraySensor_FLAG_JUNC = 0x10,
    GraySensor_FLAG_LOST = 0xFF
} GraySensor_Flag_t;

typedef struct
{
    uint8_t flag; 
    int16_t err_f; 
    int16_t err_l; 
    int16_t err_r; 

    uint8_t raw_data; 
} GraySensor_Data_t;

void GraySensor_Init(void);
void GraySensor_Update(void);
GraySensor_Data_t *GraySensor_GetData(void);
#endif