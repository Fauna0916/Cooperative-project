#include "odometry.h"
#include <math.h>
#include "utils.h"

static Odometry_State_t odo_state = {0};

static int32_t last_total_ticks_l = 0;
static int32_t last_total_ticks_r = 0;
static float meters_per_tick = 0.0f;
static float yaw_offset = 0.0f;

extern BNO080_State_t bno_state;
static float last_bno_yaw = 0.0f;
static uint8_t is_first_run = 1;


void Odometry_Init(float start_x, float start_y, float start_theta)
{
    odo_state.x = start_x;
    odo_state.y = start_y;
    odo_state.theta = start_theta;
    odo_state.distance = 0.0f;

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
    // --- Calculate Encoder Deltas ---
    int32_t current_ticks_l = Encoder_GetLeftData()->total_ticks;
    int32_t current_ticks_r = Encoder_GetRightData()->total_ticks;
    int32_t delta_ticks_l = current_ticks_l - last_total_ticks_l;
    int32_t delta_ticks_r = current_ticks_r - last_total_ticks_r;

    last_total_ticks_l = current_ticks_l;
    last_total_ticks_r = current_ticks_r;

    float dist_l = delta_ticks_l * meters_per_tick;
    float dist_r = delta_ticks_r * meters_per_tick;
    float delta_dist = (dist_r + dist_l) / 2.0f; // Linear displacement increment

    odo_state.distance += delta_dist;

    // --- Calculate Backup Heading Delta (Encoder Kinematics) ---
    // Formula: Δθ = (dr - dl) / Wheel_Track_Width
    float delta_theta_encoder = (dist_r - dist_l) / TRACK_WIDTH;

    // --- Process IMU (BNO080) Data ---
    float delta_theta = 0.0f;
    uint32_t current_tick = HAL_GetTick();

    // Check IMU health: Idle status AND heartbeat within the last 50ms
    if (bno_state == BNO080_IDLE &&
        ((uint32_t)(current_tick - BNO080_GetLatestData()->last_update_tick) < 50))
    {
        // 1Get the absolute orientation from IMU (minus the startup offset)
        float current_bno_yaw = BNO080_GetLatestData()->yaw - yaw_offset;
        current_bno_yaw = Math_NormalizeAngle(current_bno_yaw);

        if (is_first_run)
        {
            delta_theta = 0.0f;
            odo_state.theta = current_bno_yaw; // Snap to absolute
            last_bno_yaw = current_bno_yaw;
            is_first_run = 0;
        }
        else
        {
            // Calculate delta for velocity and X,Y integration
            delta_theta = Math_NormalizeAngleError(current_bno_yaw, last_bno_yaw);

            // Spike Rejection
            if (fabs(delta_theta) > 0.5f)
            {
                delta_theta = delta_theta_encoder;
                odo_state.theta = Math_NormalizeAngle(odo_state.theta + delta_theta); // Fallback to relative
            }
            else
            {
                // we set theta DIRECTLY to the IMU absolute value.
                odo_state.theta = current_bno_yaw;
            }
            last_bno_yaw = current_bno_yaw;
        }
    }
    else
    {
        // IMU Failed: Must use relative integration from encoders
        delta_theta = delta_theta_encoder;
        odo_state.theta = Math_NormalizeAngle(odo_state.theta + delta_theta);
        is_first_run = 1;
    }

    // --- Update Velocity States ---
    odo_state.linear_vel = delta_dist / ODO_UPDATE_PERIOD;
    odo_state.angular_vel = delta_theta / ODO_UPDATE_PERIOD;

    // --- Integrate X, Y using 2nd-Order Runge-Kutta (Midpoint Method) ---
    // Calculate the average heading during this time step for smoother path integration
    float avg_theta = Math_NormalizeAngle(odo_state.theta + (delta_theta / 2.0f));
    odo_state.x += delta_dist * cosf(avg_theta);
    odo_state.y += delta_dist * sinf(avg_theta);
}

Odometry_State_t *Odometry_GetState(void)
{
    return &odo_state;
}