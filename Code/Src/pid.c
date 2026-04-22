#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_integral)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->max_out = max_out;
    pid->max_integral = max_integral;

    PID_Clear(pid);
}

float PID_Compute(PID_TypeDef *pid, float target, float measured)
{
    // 1. 更新目标值与测量值
    pid->target = target;
    pid->measured = measured;

    // 2. 计算当前误差
    pid->err = pid->target - pid->measured;

    // 死区处理  ---
    if (fabsf(pid->err) < 0.2f)
        pid->err = 0;

    // 3. 积分累计
    pid->integral += pid->err;

    // --- 积分抗饱和保护 (Anti-Windup) ---
    if (pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }
    else if (pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }

    // 4. 计算 PID 输出 (位置式 PID 公式)
    // Out = Kp*e + Ki*∫e + Kd*(e - e_last)
    pid->out = (pid->Kp * pid->err) +
               (pid->Ki * pid->integral) +
               (pid->Kd * (pid->err - pid->err_last));

    // 5. 更新历史误差
    pid->err_last = pid->err;

    // --- 输出限幅保护 ---
    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    else if (pid->out < -pid->max_out)
    {
        pid->out = -pid->max_out;
    }

    return pid->out;
}

void PID_Clear(PID_TypeDef *pid)
{
    pid->target = 0;
    pid->measured = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->out = 0;
}
