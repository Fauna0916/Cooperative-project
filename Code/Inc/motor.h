#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "tim.h"

#define MOTOR_PWM_MAX 9000

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_ID_t;

void Motor_Init(void);

/**
 * @brief  
 * @param  speed_left:  [-10000, 10000]
 * @param  speed_right: [-10000, 10000]
 */
void Motor_SetSpeed(int32_t speed_left, int32_t speed_right);
void Motor_SetSingleSpeed(Motor_ID_t motor, int32_t speed);
void Motor_Brake(void);

#endif 