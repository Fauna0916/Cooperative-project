#include "encoder.h"
#include "odometry.h"

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

/* 滤波系数定义：0.0 到 1.0 之间 */
/* 值越小（如 0.2），滤波越强，速度越稳，但延迟越大 */
/* 值越大（如 0.8），响应越快，但对噪声的过滤能力越弱 */
#define SPEED_FILTER_ALPHA 0.7f

static float filtered_rpm_l = 0.0f;
static float filtered_rpm_r = 0.0f;

void Encoder_Update(void)
{
    uint16_t current_left = __HAL_TIM_GET_COUNTER(&LEFT_TIM);
    uint16_t current_right = -__HAL_TIM_GET_COUNTER(&RIGHT_TIM);

    int16_t delta_left = (int16_t)((uint16_t)current_left - (uint16_t)last_count_left);
    int16_t delta_right = (int16_t)((uint16_t)current_right - (uint16_t)last_count_right);

    last_count_left = current_left;
    last_count_right = current_right;

    // 2. 累计总里程
    left_motor.total_ticks += delta_left;
    right_motor.total_ticks += delta_right;


    float raw_rpm_l = ((float)delta_left / ENCODER_PPR) / SPEED_CALC_PERIOD * 60.0f;
    float raw_rpm_r = ((float)delta_right / ENCODER_PPR) / SPEED_CALC_PERIOD * 60.0f;

    // 4. 一阶低通滤波 (核心公式)
    // 滤波后速度 = Alpha * 本次原始速度 + (1 - Alpha) * 上次滤波速度
    filtered_rpm_l = (SPEED_FILTER_ALPHA * raw_rpm_l) + ((1.0f - SPEED_FILTER_ALPHA) * filtered_rpm_l);
    filtered_rpm_r = (SPEED_FILTER_ALPHA * raw_rpm_r) + ((1.0f - SPEED_FILTER_ALPHA) * filtered_rpm_r);

    // 5. 将滤波后的平滑数据存入结构体，供 PID 使用
    left_motor.speed_rpm = (int32_t)filtered_rpm_l;
    right_motor.speed_rpm = (int32_t)filtered_rpm_r;
}

Motor_Data_t *Encoder_GetLeftData(void)
{
    return &left_motor;
}

Motor_Data_t *Encoder_GetRightData(void)
{
    return &right_motor;
}

float Encoder_GetLinearVelocity(void)
{
    Motor_Data_t *left = Encoder_GetLeftData();
    Motor_Data_t *right = Encoder_GetRightData();

    // 转换因子：RPM 转 m/s
    float factor = (PI * WHEEL_DIAMETER) / 60.0f;

    float v_l = left->speed_rpm * factor;
    float v_r = right->speed_rpm * factor;

    return (v_r + v_l) / 2.0f;
}

float Encoder_GetAngularVelocity(void)
{
    Motor_Data_t *left = Encoder_GetLeftData();
    Motor_Data_t *right = Encoder_GetRightData();

    float factor = (PI * WHEEL_DIAMETER) / 60.0f;

    float v_l = left->speed_rpm * factor;
    float v_r = right->speed_rpm * factor;

    // 右轮快于左轮为正（左转）
    return (v_r - v_l) / TRACK_WIDTH;
}