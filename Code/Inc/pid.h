#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
} PID_PARA;

typedef struct
{
    // 1. PID parameters
    float Kp;
    float Ki;
    float Kd;

    // 2. 运行状态与数据
    float target;   // 目标值
    float measured; // 实际测量值

    float err;      // 当前误差
    float err_last; // 上一次误差
    float integral; // 误差积分累计

    // 3. 保护限幅
    float max_out;      // 输出限幅 (比如PWM最大限制 10000)
    float max_integral; // 积分限幅 (防止积分饱和，俗称"抗积分发疯"保护)

    // 4. 最终输出
    float out;
} PID_TypeDef;


/**
 * @brief  初始化 PID 参数
 * @param  pid: PID 结构体指针
 * @param  kp, ki, kd: PID 参数
 * @param  max_out: 最大输出限制
 * @param  max_integral: 最大积分限制
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_integral);

/**
 * @brief  计算 PID 输出
 * @param  pid: PID 结构体指针
 * @param  target: 当前目标值 (比如目标速度，或者目标中心点0)
 * @param  measured: 当前测量值 (比如编码器反馈，或者OpenMV的偏差)
 * @retval 计算后的控制输出量
 */
float PID_Compute(PID_TypeDef *pid, float target, float measured);

/**
 * @brief  清空 PID 历史数据（小车被抓起重放、或重新发车时调用）
 */
void PID_Clear(PID_TypeDef *pid);

#endif
