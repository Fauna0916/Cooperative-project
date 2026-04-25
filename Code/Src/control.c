#include "control.h"
#include "motor.h"
#include "encoder.h"
#include "odometry.h"
#include <math.h>
#include "utils.h"

#define MAX_LINEAR_VELOCITY 1.5f
#define MAX_ANGULAR_VELOCITY 6.0f
#define CORNERING_PENALTY_COEFF (0.15f)

// Acceleration configuration:
// If loop is 10ms (100Hz), MAX_ACCEL = 0.01f means 1.0 m/s^2 acceleration.
// Adjust this to 0.02f for 2.0 m/s^2 if the car feels too sluggish.
#define MAX_ACCEL 0.015f

PID_PARA *Tuning;

PID_PARA Velocity_loop = {8, 2.75, 0};
// PID_PARA Vision_loop = {0.0159, 0.0, 0.054};
PID_PARA Vision_loop = {0.0264, 0.0, 0.20};
PID_PARA IMU_loop = {0, 0, 0};

static PID_TypeDef pid_left_motor;  
static PID_TypeDef pid_right_motor;
static PID_TypeDef pid_line_follow; // 视觉外环 (位置/角度环)
static PID_TypeDef pid_imu_heading;
static float target_imu_yaw = 0.0f;

static Control_Mode_t current_mode = CTRL_STOP;
static float target_linear_v = 0.0f;
static float target_angular_w = 0.0f;
static float current_line_error = 0.0f;

static float current_smoothed_v = 0.0f; // Persistent state for ramping

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

static float WrapAngleError(float target, float current)
{
    float error = target - current;
    while (error > PI)
        error -= 2.0f * PI;
    while (error < -PI)
        error += 2.0f * PI;
    return error;
}

void Control_Init(void)
{
    Tuning = &IMU_loop;

    Motor_Init();
    Encoder_Init();
    Odometry_Init(0.0f, 0.0f, 0.0f);

    PID_Init(&pid_left_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);
    PID_Init(&pid_right_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);
    PID_Init(&pid_imu_heading, IMU_loop.Kp, IMU_loop.Ki, IMU_loop.Kd, 4.0f, 1.0f);

    // 输入：像素偏移，输出：角速度(rad/s)
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

        // Calculate raw target with cornering penalty
        float penalty = fabs(final_target_w) * CORNERING_PENALTY_COEFF;
        float raw_target = target_linear_v - penalty;

        if (raw_target < 0.2f)
            raw_target = 0.2f; // Minimum speed floor

        // Apply Ramping
        current_smoothed_v = Velocity_Ramp(raw_target, current_smoothed_v);
        final_target_v = current_smoothed_v;
    }
    else if (current_mode == CTRL_IMU_HEADING)
    {
        float angle_err = WrapAngleError(target_imu_yaw, Odometry_GetState()->theta);
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

void Control_SetLineError(float base_linear_vel, float openmv_error)
{
    // Only clear the PID if we are JUST transitioning into this mode
    if (current_mode != CTRL_LINE_FOLLOWING)
    {
        PID_Clear(&pid_line_follow);
    }

    current_mode = CTRL_LINE_FOLLOWING;
    target_linear_v = base_linear_vel;
    current_line_error = openmv_error;
}

void Control_SetIMUHeading(float linear_vel, float target_yaw)
{
    // Only clear the PID if we are JUST transitioning into this mode
    if (current_mode != CTRL_IMU_HEADING)
    {
        PID_Clear(&pid_imu_heading);
    }

    current_mode = CTRL_IMU_HEADING;
    target_linear_v = linear_vel;

    while (target_yaw > PI)
        target_yaw -= 2.0f * PI;
    while (target_yaw < -PI)
        target_yaw += 2.0f * PI;

    target_imu_yaw = target_yaw;
}

bool Control_IsHeadingSettled(void)
{
    if (current_mode != CTRL_IMU_HEADING)
        return false;

    float current_yaw = Odometry_GetState()->theta;
    float angle_err = WrapAngleError(target_imu_yaw, current_yaw);
    float current_w = Odometry_GetState()->angular_vel; // Assuming odometry tracks w

    // If error is less than 2 degrees (0.035 rad) AND robot has mostly stopped spinning
    if (fabs(angle_err) < 0.035f && fabs(current_w) < 0.1f)
    {
        return true;
    }
    return false;
}