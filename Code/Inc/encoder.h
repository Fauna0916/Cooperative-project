#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "tim.h"

// 轮子转动一圈的总脉冲数 (13线 * 20减速比 * 4倍频)
#define ENCODER_PPR 1040.0f

// 测速周期，单位秒
#define SPEED_CALC_PERIOD 0.01f

typedef struct
{
    int32_t speed_rpm;   // 真实转速 (转/分钟)
    int32_t total_ticks; // 累计总路程脉冲数
} Motor_Data_t;

void Encoder_Init(void);
void Encoder_Update(void);

Motor_Data_t *Encoder_GetLeftData(void);
Motor_Data_t *Encoder_GetRightData(void);

/**
 * @brief  基于编码器计算当前的理论
 *
 */
float Encoder_GetLinearVelocity(void);
float Encoder_GetAngularVelocity(void);

#endif
