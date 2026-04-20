#ifndef __CALLBACK_H__
#define __CALLBACK_H__

#include "tim.h"
#include "bno080.h"
#include "encoder.h"
#include "usart.h"
#include "odometry.h"

enum
{
    False = 0,
    True = 1,
};

void UART2_handle(void);

#endif