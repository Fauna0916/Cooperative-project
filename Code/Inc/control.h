#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "pid.h"

typedef enum
{
    CTRL_STOP = 0,      // 停车模式 (关闭电机，清空PID)
    CTRL_SPEED_MODE,    // 纯速度控制模式 (指定线速度和角速度，用于盲走/遥控)
    CTRL_LINE_FOLLOWING // 视觉巡线模式 (使用 OpenMV 反馈闭环)
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

/**
 * @brief  设置巡线模式 (CTRL_LINE_FOLLOWING 模式)
 * @param  base_linear_vel: 基础前行速度 (m/s)
 * @param  openmv_error: OpenMV传回的黑线偏移量 (例如 -100~100，0为正中心)
 */
void Control_SetLineError(float base_linear_vel, float openmv_error);

#endif
