#ifndef __DS3231_H
#define __DS3231_H

#include "i2c.h"

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} DS3231_Time;

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time *time);

HAL_StatusTypeDef DS3231_SetTime(uint8_t year,
                                 uint8_t month,
                                 uint8_t date,
                                 uint8_t hours,
                                 uint8_t minutes,
                                 uint8_t seconds);

#endif