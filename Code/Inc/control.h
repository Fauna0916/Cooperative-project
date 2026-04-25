#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "pid.h"
#include <stdbool.h>

typedef enum
{
    CTRL_STOP = 0,       
    CTRL_SPEED_MODE,    
    CTRL_LINE_FOLLOWING, 
    CTRL_IMU_HEADING,
} Control_Mode_t;

extern PID_PARA *Tuning;

void Control_Init(void);
void Control_Update(void);
void Control_Stop(void);

/**
 * @brief  设置小车的目标速度 (CTRL_SPEED_MODE 模式)
 * @param  linear_vel: 线速度 (米/秒, m/s)
 * @param  angular_vel: 角速度 (弧度/秒, rad/s)，正数为左转，负数为右转
 */
void Control_SetVelocity(float linear_vel, float angular_vel);
void Control_SetLineError(float base_linear_vel, float openmv_error);
void Control_SetIMUHeading(float linear_vel, float target_yaw);
bool Control_IsHeadingSettled(void);

#endif
