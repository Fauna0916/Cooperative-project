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

    // 3. 将脉冲增量转换为物理距离增量 (米)
    float dist_l = delta_ticks_l * meters_per_tick;
    float dist_r = delta_ticks_r * meters_per_tick;

    // 4. 计算中心点移动距离和车体旋转角度增量
    float delta_dist = (dist_r + dist_l) / 2.0f;         // 小车中心行走的距离
    float current_yaw = BNO080_GetLatestData()->yaw - yaw_offset;

    // 角度归一化到 -PI ~ PI
    if (current_yaw > PI)
        current_yaw -= 2.0f * PI;
    if (current_yaw < -PI)
        current_yaw += 2.0f * PI;


    // 更新瞬时速度 (m/s 和 rad/s)
    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = (current_yaw - odo_state.theta) / ODO_UPDATE_PERIOD;


    odo_state.theta = current_yaw;
    odo_state.x += delta_dist * cosf(odo_state.theta);
    odo_state.y += delta_dist * sinf(odo_state.theta);
}

Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}