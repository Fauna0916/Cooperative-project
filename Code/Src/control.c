#include "control.h"
#include "motor.h"
#include "encoder.h"
#include "odometry.h"
#include <math.h>
#include "utils.h"

#define MAX_LINEAR_VELOCITY 1.5f
#define MAX_ANGULAR_VELOCITY 6.0f
#define CORNERING_PENALTY_COEFF (0.15f)
#define HEADING_SETTLE_TIMEOUT 2000 // ms

PID_PARA *Tuning;

// PID_PARA Velocity_loop = {8, 2.75, 0.0};    //outdoors
PID_PARA Velocity_loop = {20, 2.06, 0.0}; // indoors


// PID_PARA Vision_loop = {0.0215, 0.0, 0.045};
PID_PARA Vision_loop = {0.01, 0.0, 0.15};

PID_PARA IMU_loop = {2.71, 0.0, 0.08}; // outdorrs

static PID_TypeDef pid_left_motor;
static PID_TypeDef pid_right_motor;
static PID_TypeDef pid_line_follow;
static PID_TypeDef pid_imu_heading;

static Control_Mode_t current_mode = CTRL_STOP;
static float target_linear_v = 0.0f;
static float target_angular_w = 0.0f;
static float current_line_error = 0.0f;

/*ramp*/
#define MAX_ACCEL 0.015f              // loop is 10ms(100Hz), MAX_ACCEL = 0.01f means 1.0 m / s ^ 2 acceleration.
#define YAW_RAMP_SPEED_DEFAULT 0.015f // 0.01 rad per 10ms = 1.0 rad/s (approx 57 deg/s)

static float current_smoothed_v = 0.0f; // Persistent state for ramping

static float ramp_target_yaw = 0.0f;  // The moving target the PID follows
static float final_target_yaw = 0.0f; // The ultimate destination
static float yaw_ramp_step = 0.01f;   // How much the angle increases per 10ms (Speed)

static float Velocity_Ramp(float target, float current)
{
    if (target > current + MAX_ACCEL)
    {
        return current + MAX_ACCEL;
    }
    else if (target < current - MAX_ACCEL)
    {
        return current - MAX_ACCEL;
    }
    else
    {
        return target;
    }
}

/**
 * @brief  Updates the ramp setpoint towards the final target
 * @return 1 if ramp reached final target, 0 otherwise
 */
uint8_t Update_Yaw_Ramp(void)
{
    float error = Math_NormalizeAngleError(final_target_yaw, ramp_target_yaw);

    if (fabs(error) < yaw_ramp_step)
    {
        ramp_target_yaw = final_target_yaw;
        return 1; // Reached
    }
    else
    {
        // Move ramp_target closer to final_target by one small step
        if (error > 0)
            ramp_target_yaw += yaw_ramp_step;
        else
            ramp_target_yaw -= yaw_ramp_step;

        // Keep ramp_target in -PI to PI range
        ramp_target_yaw = Math_NormalizeAngle(ramp_target_yaw);

        return 0; // Still moving
    }
}

void Control_Init(void)
{
    Tuning = &Vision_loop;

    Motor_Init();
    Encoder_Init();
    Odometry_Init(0.0f, 0.0f, 0.0f);

    PID_Init(&pid_left_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);
    PID_Init(&pid_right_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);
    PID_Init(&pid_imu_heading, IMU_loop.Kp, IMU_loop.Ki, IMU_loop.Kd, 4.0f, 1.0f);

    PID_Init(&pid_line_follow, Vision_loop.Kp, Vision_loop.Ki, Vision_loop.Kd, 4.0f, 1.0f);

    current_mode = CTRL_STOP;
}

/**
 * @brief  运动学逆解算：把线速度和角速度，转换为左右轮的目标转速 (RPM)
 */
static void Kinematics_VelocityToRPM(float linear_v, float angular_w, float *rpm_l, float *rpm_r)
{
    // 差速底盘公式：
    // v_left = v - (w * TRACK_WIDTH) / 2
    // v_right = v + (w * TRACK_WIDTH) / 2
    float v_l = linear_v - (angular_w * TRACK_WIDTH) / 2.0f;
    float v_r = linear_v + (angular_w * TRACK_WIDTH) / 2.0f;

    // 转换公式：v (m/s) -> 圈/秒 -> 圈/分 (RPM)
    // 轮子周长 = PI * WHEEL_DIAMETER
    static const float conversion_factor = 60.0f / (PI * WHEEL_DIAMETER);

    *rpm_l = v_l * conversion_factor;
    *rpm_r = v_r * conversion_factor;
}

void Control_Update(void)
{
    pid_left_motor.Kp = pid_right_motor.Kp = Velocity_loop.Kp;
    pid_left_motor.Ki = pid_right_motor.Ki = Velocity_loop.Ki;
    pid_left_motor.Kd = pid_right_motor.Kd = Velocity_loop.Kd;

    pid_imu_heading.Kp = IMU_loop.Kp;
    pid_imu_heading.Ki = IMU_loop.Ki;
    pid_imu_heading.Kd = IMU_loop.Kd;

    pid_line_follow.Kp = Vision_loop.Kp;
    pid_line_follow.Ki = Vision_loop.Ki;
    pid_line_follow.Kd = Vision_loop.Kd;

    Encoder_Update();
    Odometry_Update();

    // Handle Stop Condition
    if (current_mode == CTRL_STOP)
    {
        current_smoothed_v = 0.0f; // Reset ramp state
        Motor_SetSpeed(0, 0);
        PID_Clear(&pid_left_motor);
        PID_Clear(&pid_right_motor);
        PID_Clear(&pid_line_follow);
        return;
    }

    float final_target_v = 0.0f; // This will hold the ramped velocity
    float final_target_w = target_angular_w;

    // 3Decision Layer: Mode Selection
    if (current_mode == CTRL_LINE_FOLLOWING)
    {
        final_target_w = PID_Compute(&pid_line_follow, 0.0f, current_line_error);

        float raw_target = target_linear_v;

        const float MIN_FORWARD_SPEED = 0.01f; // m/s

        if (target_linear_v > 0)
        {
            if (raw_target < MIN_FORWARD_SPEED)
            {
                raw_target = MIN_FORWARD_SPEED;
            }
        }
        else if (target_linear_v < 0)
        {
            // If we are reversing, ensure we don't accidentally start moving forward
            if (raw_target > -MIN_FORWARD_SPEED)
            {
                raw_target = -MIN_FORWARD_SPEED;
            }
        }
        else
        {
            raw_target = 0;
        }

        // Apply Ramping
        current_smoothed_v = Velocity_Ramp(raw_target, current_smoothed_v);
        final_target_v = current_smoothed_v;
    }
    else if (current_mode == CTRL_IMU_HEADING)
    {
        Update_Yaw_Ramp();

        float angle_err = Math_NormalizeAngleError(ramp_target_yaw, Odometry_GetState()->theta);
        final_target_w = PID_Compute(&pid_imu_heading, 0.0f, -angle_err);

        current_smoothed_v = Velocity_Ramp(target_linear_v, current_smoothed_v);
        final_target_v = current_smoothed_v;
    }
    else if (current_mode == CTRL_SPEED_MODE)
    {
        current_smoothed_v = Velocity_Ramp(target_linear_v, current_smoothed_v);
        final_target_v = current_smoothed_v;
    }

    // Kinematics Layer: Convert (v, w) -> (rpm_l, rpm_r)
    float target_rpm_l, target_rpm_r;
    Kinematics_VelocityToRPM(final_target_v, final_target_w, &target_rpm_l, &target_rpm_r);

    // Execution Layer: Inner Speed Loop
    float measured_rpm_l = Encoder_GetLeftData()->speed_rpm;
    float measured_rpm_r = Encoder_GetRightData()->speed_rpm;

    float pwm_l = PID_Compute(&pid_left_motor, target_rpm_l, measured_rpm_l);
    float pwm_r = PID_Compute(&pid_right_motor, target_rpm_r, measured_rpm_r);

    Motor_SetSpeed((int32_t)pwm_l, (int32_t)pwm_r);
}

void Control_Stop(void)
{
    current_mode = CTRL_STOP;
}

void Control_SetVelocity(float linear_vel, float angular_vel)
{
    current_mode = CTRL_SPEED_MODE;

    if (linear_vel > MAX_LINEAR_VELOCITY)
        linear_vel = MAX_LINEAR_VELOCITY;
    if (linear_vel < -MAX_LINEAR_VELOCITY)
        linear_vel = -MAX_LINEAR_VELOCITY;

    if (angular_vel > MAX_ANGULAR_VELOCITY)
        angular_vel = MAX_ANGULAR_VELOCITY;
    if (angular_vel < -MAX_ANGULAR_VELOCITY)
        angular_vel = -MAX_ANGULAR_VELOCITY;

    target_linear_v = linear_vel;
    target_angular_w = angular_vel;
}

void Control_SetLineError(float base_linear_vel, float error)
{
    // Only clear the PID if we are JUST transitioning into this mode
    if (current_mode != CTRL_LINE_FOLLOWING)
    {
        PID_Clear(&pid_line_follow);
    }

    current_mode = CTRL_LINE_FOLLOWING;
    target_linear_v = base_linear_vel;
    current_line_error = error;
}

void Control_SetIMUHeading(float linear_vel, float target_yaw)
{
    target_yaw = Math_NormalizeAngle(target_yaw);

    // 如果目标改变或模式切换，重置PID
    if (current_mode != CTRL_IMU_HEADING || fabs(Math_NormalizeAngleError(target_yaw, final_target_yaw)) > 0.02f)
    {
        PID_Clear(&pid_imu_heading);
        final_target_yaw = target_yaw; // 直接记录最终目标
    }

    current_mode = CTRL_IMU_HEADING;
    target_linear_v = linear_vel;
    // 删除了 yaw_ramp_step 相关逻辑
}

bool Control_IsHeadingSettled(void)
{
    if (current_mode != CTRL_IMU_HEADING)
        return false;

    static uint32_t start_tick = 0;
    static float last_target = -999.0f;

    if (last_target != final_target_yaw)
    {
        start_tick = HAL_GetTick();
        last_target = final_target_yaw;
    }

    float current_yaw = Odometry_GetState()->theta;
    float angle_err = Math_NormalizeAngleError(final_target_yaw, current_yaw);
    float current_w = Odometry_GetState()->angular_vel;

    // If error is less than 6 degrees (0.035 rad) AND robot has mostly stopped spinning
    if (fabs(angle_err) < 0.105f && fabs(current_w) < 0.1f)
    {
        printf("settleted\r\n");
        start_tick = 0;
        return true;
    }

    // timeout
    if (HAL_GetTick() - start_tick > HEADING_SETTLE_TIMEOUT)
    {
        start_tick = 0;
        return true;
    }
    return false;
}