#include "odometry.h"


static Odometry_State_t odo_state = {0};

static int32_t last_total_ticks_l = 0;
static int32_t last_total_ticks_r = 0;

static float meters_per_tick = 0.0f;

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
    float delta_theta = (dist_r - dist_l) / TRACK_WIDTH; // 小车旋转的角度 (弧度)

    // 5. 更新瞬时速度 (m/s 和 rad/s)
    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = delta_theta / ODO_UPDATE_PERIOD;

    // 6. 坐标系转换：计算世界坐标系下的位置增量 (使用二阶近似提高弯道精度)
    // 相比使用旧角度，使用 (旧角度 + 旋转量的一半) 更加接近圆弧运动的真实轨迹
    float average_theta = odo_state.theta + (delta_theta / 2.0f);

    odo_state.x += delta_dist * cosf(average_theta);
    odo_state.y += delta_dist * sinf(average_theta);
    odo_state.theta += delta_theta;

    // 7. 角度归一化，将 theta 限制在 -PI 到 PI 之间
    if (odo_state.theta > PI)
    {
        odo_state.theta -= 2.0f * PI;
    }
    else if (odo_state.theta < -PI)
    {
        odo_state.theta += 2.0f * PI;
    }
}

Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}