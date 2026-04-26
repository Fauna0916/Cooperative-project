#ifndef __BNO080_H__
#define __BNO080_H__

#include "i2c.h"

#define BNO080_I2C hi2c1
#define BNO080_I2C_ADDR 0x96    // 0x4B << 1

#define BNO_READ_SIZE 32
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08

typedef enum
{
    BNO080_IDLE = 0,
    BNO080_READING,
    BNO080_DATA_READY,
    BNO080_READY_TO_READ,
    BNO080_ERROR = -1,
} BNO080_State_t;

typedef struct 
{
    float i, j, k, real;
    float yaw, pitch, roll;
    uint8_t quat_accuracy;
    uint32_t last_update_tick;
} BNO080_Data_t;


void BNO080_Init(void);
BNO080_State_t BNO080_Update(void);
BNO080_Data_t *BNO080_GetLatestData(void);
void BNO_EXTI_Callback(uint16_t GPIO_Pin);
void BNO_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void BNO_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif