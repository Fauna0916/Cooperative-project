#include "encoder.h"

#define LEFT_TIM (htim2)
#define RIGHT_TIM (htim3)

static Motor_Data_t left_motor = {0, 0};
static Motor_Data_t right_motor = {0, 0};


static uint16_t last_count_left = 0;
static uint16_t last_count_right = 0;

void Encoder_Init(void)
{

    HAL_TIM_Encoder_Start(&LEFT_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&RIGHT_TIM, TIM_CHANNEL_ALL);

    // init val
    last_count_left = __HAL_TIM_GET_COUNTER(&LEFT_TIM);
    last_count_right = __HAL_TIM_GET_COUNTER(&RIGHT_TIM);
}

void Encoder_Update(void)
{
    uint16_t current_left = __HAL_TIM_GET_COUNTER(&LEFT_TIM);
    uint16_t current_right = __HAL_TIM_GET_COUNTER(&RIGHT_TIM);


    // 正数=正转，负数=反转。
    int16_t delta_left = (int16_t)(current_left - last_count_left);
    int16_t delta_right = (int16_t)(current_right - last_count_right);

    last_count_left = current_left;
    last_count_right = current_right;

    // 累计里程 (可选功能)
    left_motor.total_ticks += delta_left;
    right_motor.total_ticks += delta_right;

    // 3. 计算真实 RPM (转/分钟)
    // 算法: (周期脉冲数 / 单圈总脉冲数) / 测速周期 * 60秒
    left_motor.speed_rpm = (int32_t)(((float)delta_left / ENCODER_PPR) / SPEED_CALC_PERIOD * 60.0f);
    right_motor.speed_rpm = (int32_t)(((float)delta_right / ENCODER_PPR) / SPEED_CALC_PERIOD * 60.0f);

}

Motor_Data_t *Encoder_GetLeftData(void)
{
    return &left_motor;
}

Motor_Data_t *Encoder_GetRightData(void)
{
    return &right_motor;
}