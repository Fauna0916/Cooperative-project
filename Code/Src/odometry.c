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
 * @brief  Wraps the given angle to the range [-PI, PI].
 */
static inline float Math_NormalizeAngle(float angle)
{
    float a = fmodf(angle + PI, 2.0f * PI);
    if (a < 0)
        a += 2.0f * PI;
    return a - PI;
}

/**
 * @brief  Calculates the shortest-path angle error between target and current.
 * @note   Used for PID error or delta calculations to prevent "long-way-around"
 *         rotations (e.g., from 179 deg to -179 deg).
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

    // Capture initial encoder counts as the baseline reference
    last_total_ticks_l = Encoder_GetLeftData()->total_ticks;
    last_total_ticks_r = Encoder_GetRightData()->total_ticks;

    // Physical distance per pulse = Wheel Circumference / Pulses Per Revolution
    meters_per_tick = (PI * WHEEL_DIAMETER) / ENCODER_PPR;

    // Synchronize IMU yaw with the robot's starting orientation
    yaw_offset = BNO080_GetLatestData()->yaw - start_theta;
}

void Odometry_Update(void)
{
    // --- 1. Calculate Encoder Deltas ---
    int32_t current_ticks_l = Encoder_GetLeftData()->total_ticks;
    int32_t current_ticks_r = Encoder_GetRightData()->total_ticks;
    int32_t delta_ticks_l = current_ticks_l - last_total_ticks_l;
    int32_t delta_ticks_r = current_ticks_r - last_total_ticks_r;

    last_total_ticks_l = current_ticks_l;
    last_total_ticks_r = current_ticks_r;

    float dist_l = delta_ticks_l * meters_per_tick;
    float dist_r = delta_ticks_r * meters_per_tick;
    float delta_dist = (dist_r + dist_l) / 2.0f; // Linear displacement increment

    // --- 2. Calculate Backup Heading Delta (Encoder Kinematics) ---
    // Formula: Δθ = (dr - dl) / Wheel_Track_Width
    float delta_theta_encoder = (dist_r - dist_l) / TRACK_WIDTH;

    // --- 3. Process IMU (BNO080) Data ---
    float delta_theta = 0.0f;
    uint32_t current_tick = HAL_GetTick();

    // Check IMU health: Idle status AND heartbeat within the last 50ms
    if (bno_state == BNO080_IDLE &&
        ((uint32_t)(current_tick - BNO080_GetLatestData()->last_update_tick) < 50))
    {
        float current_bno_yaw = BNO080_GetLatestData()->yaw - yaw_offset;

        if (is_first_run)
        {
            // First frame after initialization or recovery:
            // Use encoder delta as a bridge while resyncing the IMU baseline
            delta_theta = delta_theta_encoder;
            last_bno_yaw = current_bno_yaw;
            is_first_run = 0;
        }
        else
        {
            // Normal operation: Compute heading change from IMU
            delta_theta = Math_NormalizeAngleError(current_bno_yaw, last_bno_yaw);

            // Spike Rejection: If change > 0.5 rad (~30 deg) in 10ms, assume interference
            if (fabs(delta_theta) > 0.5f)
            {
                delta_theta = delta_theta_encoder; // Fallback to encoders
            }

            last_bno_yaw = current_bno_yaw;
        }
    }
    else
    {
        // IMU Failed or Timed out: Rely entirely on encoders
        delta_theta = delta_theta_encoder;
        is_first_run = 1; // Flag for resync when IMU returns
    }

    // --- 4. Update Velocity States ---
    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = delta_theta / ODO_UPDATE_PERIOD;

    // --- 5. Integrate X, Y using 2nd-Order Runge-Kutta (Midpoint Method) ---
    // Calculate the average heading during this time step for smoother path integration
    float avg_theta = Math_NormalizeAngle(odo_state.theta + (delta_theta / 2.0f));
    odo_state.x += delta_dist * cosf(avg_theta);
    odo_state.y += delta_dist * sinf(avg_theta);

    odo_state.theta = Math_NormalizeAngle(odo_state.theta + delta_theta);
}

Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}