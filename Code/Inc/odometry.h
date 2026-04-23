#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "encoder.h"
#include "bno080.h"
#include <math.h>

// 轮子直径，单位：米
#define WHEEL_DIAMETER 0.045f

// 两轮中心间距 (轮距)，单位：米
#define TRACK_WIDTH 0.148f

// 13 * 20 * 4 = 1040
#define ENCODER_PPR 1040.0f
#define ODO_UPDATE_PERIOD 0.01f // s

#define PI 3.1415926535f

typedef struct
{
    // 位置与姿态 (世界坐标系)
    float x;     // X 坐标，单位：米
    float y;     // Y 坐标，单位：米
    float theta; // 偏航角，单位：弧度 (范围 -PI 到 PI)

    // 瞬时速度 (机器人坐标系)
    float linear_vel;  // 线速度，单位：米/秒 (m/s)
    float angular_vel; // 角速度，单位：弧度/秒 (rad/s)
} Odometry_State_t;

/**
 * @brief  初始化/重置里程计
 * @param  start_x: 初始 X 坐标 (通常为 0)
 * @param  start_y: 初始 Y 坐标 (通常为 0)
 * @param  start_theta: 初始朝向角 (通常为 0)
 */
void Odometry_Init(float start_x, float start_y, float start_theta);
void Odometry_Update(void);
Odometry_State_t *Odometry_GetState(void);

#endif
