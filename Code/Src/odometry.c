#include "odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static Odometry_State_t odo_state = {0};

static int32_t last_total_ticks_l = 0;
static int32_t last_total_ticks_r = 0;
static float meters_per_tick = 0.0f;
static float yaw_offset = 0.0f;

extern BNO080_State_t bno_state;
static float last_bno_yaw = 0.0f;
static uint8_t is_first_run = 1;

/**
 * @brief  将角度限制在 [-PI, PI] 之间
 */
static inline float Math_NormalizeAngle(float angle)
{
    float a = fmodf(angle + PI, 2.0f * PI);
    if (a < 0)
        a += 2.0f * PI;
    return a - PI;
}

/**
 * @brief  计算最短路径的角度误差，并限制在 [-PI, PI]
 * @note   专门用于 PID 误差计算，防止从 179度 转到 -179度 时走远路
 */
static inline float Math_NormalizeAngleError(float target, float current)
{
    float err = target - current;
    return Math_NormalizeAngle(err);
}

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
    float delta_dist = (dist_r + dist_l) / 2.0f; // 线距离增量

    // 2. 计算【备份用】的编码器角度增量
    // 公式：Δθ = (dr - dl) / L
    float delta_theta_encoder = (dist_r - dist_l) / TRACK_WIDTH;

    // 3. 处理 BNO080 数据
    float delta_theta = 0.0f;

    // 检查 BNO080 是否健康 (假设 BNO080_Update 会在报错时改变状态)
    // 并且检查时间戳，如果超过 50ms 没更新，视为失效
    if (bno_state == BNO080_IDLE && (HAL_GetTick() - BNO080_GetLatestData()->last_update_tick < 50))
    {
        float current_bno_yaw = BNO080_GetLatestData()->yaw - yaw_offset;

        if (is_first_run)
        {
            last_bno_yaw = current_bno_yaw;
            is_first_run = 0;
        }

        delta_theta = Math_NormalizeAngleError(current_bno_yaw, last_bno_yaw);

        // 增量过大判定 (瞬间跳变保护)：如果一帧(10ms)跳变超过 0.5 弧度(约30度)，判定为干扰
        if (fabs(delta_theta) > 0.5f)
        {
            delta_theta = delta_theta_encoder; // 此时切换到编码器增量
        }

        last_bno_yaw = current_bno_yaw; // 更新历史记录
    }
    else
    {
        // 【重置期间/传感器失效】：完全信任编码器
        delta_theta = delta_theta_encoder;
        // 同时标记 is_first_run，以便 BNO 恢复时重新同步
        is_first_run = 1;
    }

    // 4. 角度增量归一化处理
    while (delta_theta > PI)
        delta_theta -= 2.0f * PI;
    while (delta_theta < -PI)
        delta_theta += 2.0f * PI;

    // 5. 更新状态量
    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = delta_theta / ODO_UPDATE_PERIOD;

    // 使用二阶龙格库塔积分更新 X, Y
    float avg_theta = Math_NormalizeAngle(odo_state.theta + (delta_theta / 2.0f));
    odo_state.x += delta_dist * cosf(avg_theta);
    odo_state.y += delta_dist * sinf(avg_theta);

    odo_state.theta += delta_theta;

    odo_state.theta = Math_NormalizeAngle(odo_state.theta + delta_theta);
}

Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}