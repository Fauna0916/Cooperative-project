#ifndef __UTILS_H__
#define __UTILS_H__

#include "stdio.h"
#include "usart.h"
#include "callback.h"

void I2C_VerifyCommunication(I2C_HandleTypeDef *device_I2C, uint16_t device_address);
void debug_info(void);
void Tuning_Init(void);

#endif