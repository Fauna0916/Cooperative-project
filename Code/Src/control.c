#include "control.h"
#include "motor.h"
#include "encoder.h"
#include "odometry.h"
#include <math.h>
#include "utils.h"


#define MAX_LINEAR_VELOCITY 1.2f  
#define MAX_ANGULAR_VELOCITY 6.0f


PID_PARA *Tuning;

PID_PARA Velocity_loop = {8, 2.75, 0};
PID_PARA Vision_loop = {0, 0, 0};

static PID_TypeDef pid_left_motor;  // 左轮内环 (速度环)
static PID_TypeDef pid_right_motor; // 右轮内环 (速度环)
static PID_TypeDef pid_line_follow; // 视觉外环 (位置/角度环)

static Control_Mode_t current_mode = CTRL_STOP;
static float target_linear_v = 0.0f;
static float target_angular_w = 0.0f;
static float current_line_error = 0.0f;

void Control_Init(void)
{
    Tuning = &Velocity_loop;

    Motor_Init();
    Encoder_Init();
    Odometry_Init(0.0f, 0.0f, 0.0f);

    PID_Init(&pid_left_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);
    PID_Init(&pid_right_motor, Velocity_loop.Kp, Velocity_loop.Ki, Velocity_loop.Kd, 9500.0f, 3000.0f);

    // 4. 初始化视觉巡线 PID
    // 输入：像素偏移，输出：角速度(rad/s)
    // 经验值：Kp=0.02, Ki=0.0, Kd=0.05
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
    const float conversion_factor = 60.0f / (PI * WHEEL_DIAMETER);

    *rpm_l = v_l * conversion_factor;
    *rpm_r = v_r * conversion_factor;
}

void Control_Update(void)
{
    pid_left_motor.Kp = pid_right_motor.Kp = Velocity_loop.Kp;
    pid_left_motor.Ki = pid_right_motor.Ki = Velocity_loop.Ki;
    pid_left_motor.Kd = pid_right_motor.Kd = Velocity_loop.Kd;

    Encoder_Update();
    Odometry_Update();

    if (current_mode == CTRL_STOP)
    {
        Motor_SetSpeed(0, 0);
        PID_Clear(&pid_left_motor);
        PID_Clear(&pid_right_motor);
        PID_Clear(&pid_line_follow);
        return;
    }

    float target_rpm_l = 0.0f;
    float target_rpm_r = 0.0f;

    // 2. 决策层：计算外环输出
    if (current_mode == CTRL_LINE_FOLLOWING)
    {
        // 视觉外环计算：目标是把偏差(current_line_error)消除为0
        // 外环的输出值，就是小车需要的角速度 (rad/s)
        target_angular_w = PID_Compute(&pid_line_follow, 0.0f, current_line_error);

        // 逆解算转换为电机 RPM
        Kinematics_VelocityToRPM(target_linear_v, target_angular_w, &target_rpm_l, &target_rpm_r);
    }
    else if (current_mode == CTRL_SPEED_MODE)
    {
        // 纯速度控制模式，直接解算
        Kinematics_VelocityToRPM(target_linear_v, target_angular_w, &target_rpm_l, &target_rpm_r);
    }

    // 3. 执行层：内环电机速度闭环
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
    current_mode = CTRL_LINE_FOLLOWING;
    target_linear_v = base_linear_vel;
    current_line_error = openmv_error;
}