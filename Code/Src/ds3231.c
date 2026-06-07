#include "ds3231.h"

#define DS3231_ADDR (0x68 << 1)

extern I2C_HandleTypeDef hi2c2;

static uint8_t BCD2DEC(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t DEC2BCD(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

HAL_StatusTypeDef DS3231_GetTime(DS3231_Time *time)
{
    uint8_t data[7];

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c2,
                                             DS3231_ADDR,
                                             0x00,
                                             I2C_MEMADD_SIZE_8BIT,
                                             data,
                                             7,
                                             100);

    if (ret != HAL_OK)
    {
        return ret;
    }

    time->seconds = BCD2DEC(data[0] & 0x7F);
    time->minutes = BCD2DEC(data[1] & 0x7F);
    time->hours   = BCD2DEC(data[2] & 0x3F);
    time->day     = BCD2DEC(data[3]);
    time->date    = BCD2DEC(data[4]);
    time->month   = BCD2DEC(data[5] & 0x1F);
    time->year    = BCD2DEC(data[6]);

    return HAL_OK;
}

HAL_StatusTypeDef DS3231_SetTime(uint8_t year,
                                 uint8_t month,
                                 uint8_t date,
                                 uint8_t hours,
                                 uint8_t minutes,
                                 uint8_t seconds)
{
    uint8_t data[7];

    data[0] = DEC2BCD(seconds);
    data[1] = DEC2BCD(minutes);
    data[2] = DEC2BCD(hours);
    data[3] = DEC2BCD(1);
    data[4] = DEC2BCD(date);
    data[5] = DEC2BCD(month);
    data[6] = DEC2BCD(year);

    return HAL_I2C_Mem_Write(&hi2c2,
                             DS3231_ADDR,
                             0x00,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             7,
                             100);
}