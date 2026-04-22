#include "odometry.h"


static Odometry_State_t odo_state = {0};

static int32_t last_total_ticks_l = 0;
static int32_t last_total_ticks_r = 0;
static float meters_per_tick = 0.0f;

static float yaw_offset = 0.0f;

void Odometry_Init(float start_x, float start_y, float start_theta)
{
    odo_state.x = start_x;
    odo_state.y = start_y;
    odo_state.theta = start_theta;

    odo_state.linear_vel = 0.0f;
    odo_state.angular_vel = 0.0f;

    // 初始化时获取当前编码器的累计值，作为基准点
    last_total_ticks_l = Encoder_GetLeftData()->total_ticks;
    last_total_ticks_r = Encoder_GetRightData()->total_ticks;

    // 计算每个脉冲代表的物理距离 (米/脉冲) = 轮子周长 / 单圈脉冲数
    meters_per_tick = (PI * WHEEL_DIAMETER) / ENCODER_PPR;

    yaw_offset = BNO080_GetLatestData()->yaw - start_theta;
}

void Odometry_Update(void)
{
    int32_t current_ticks_l = Encoder_GetLeftData()->total_ticks;
    int32_t current_ticks_r = Encoder_GetRightData()->total_ticks;
    int32_t delta_ticks_l = current_ticks_l - last_total_ticks_l;
    int32_t delta_ticks_r = current_ticks_r - last_total_ticks_r;
    last_total_ticks_l = current_ticks_l;
    last_total_ticks_r = current_ticks_r;

    float dist_l = delta_ticks_l * meters_per_tick;
    float dist_r = delta_ticks_r * meters_per_tick;
    float delta_dist = (dist_r + dist_l) / 2.0f;

    float raw_yaw = BNO080_GetLatestData()->yaw - yaw_offset;

    while (raw_yaw > PI)
        raw_yaw -= 2.0f * PI;
    while (raw_yaw < -PI)
        raw_yaw += 2.0f * PI;
    float current_yaw = raw_yaw;

    float delta_theta = current_yaw - odo_state.theta;
    if (delta_theta > PI)
        delta_theta -= 2.0f * PI;
    else if (delta_theta < -PI)
        delta_theta += 2.0f * PI;

    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = delta_theta / ODO_UPDATE_PERIOD;


    float avg_theta = odo_state.theta + (delta_theta / 2.0f);
    odo_state.x += delta_dist * cosf(avg_theta);
    odo_state.y += delta_dist * sinf(avg_theta);

    odo_state.theta = current_yaw;
}
Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}