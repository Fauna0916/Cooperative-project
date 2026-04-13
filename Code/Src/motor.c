#include "motor.h"

#define MOTOR_TIM htim4

#define LEFT_IN1_CH TIM_CHANNEL_1
#define LEFT_IN2_CH TIM_CHANNEL_2
#define RIGHT_IN1_CH TIM_CHANNEL_3
#define RIGHT_IN2_CH TIM_CHANNEL_4

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR_TIM, LEFT_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_TIM, LEFT_IN2_CH);
    HAL_TIM_PWM_Start(&MOTOR_TIM, RIGHT_IN1_CH);
    HAL_TIM_PWM_Start(&MOTOR_TIM, RIGHT_IN2_CH);

    Motor_SetSpeed(0, 0);
}

void Motor_SetSingleSpeed(Motor_ID_t motor, int32_t speed)
{
    if (speed > MOTOR_PWM_MAX)
        speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX)
        speed = -MOTOR_PWM_MAX;


    if (motor == MOTOR_LEFT)
    {
        if (speed > 0)
        {

            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN1_CH, speed);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN2_CH, 0);
        }
        else if (speed < 0)
        {

            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN2_CH, -speed);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN2_CH, 0);
        }
    }
    else if (motor == MOTOR_RIGHT)
    {
        if (speed > 0)
        {
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN1_CH, speed);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN2_CH, 0);
        }
        else if (speed < 0)
        {
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN2_CH, -speed);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN1_CH, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN2_CH, 0);
        }
    }
}

void Motor_SetSpeed(int32_t speed_left, int32_t speed_right)
{
    Motor_SetSingleSpeed(MOTOR_LEFT, speed_left);
    Motor_SetSingleSpeed(MOTOR_RIGHT, speed_right);
}

void Motor_Brake(void)
{
    // set IN1, IN2 high
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN1_CH, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, LEFT_IN2_CH, MOTOR_PWM_MAX);

    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN1_CH, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, RIGHT_IN2_CH, MOTOR_PWM_MAX);
}