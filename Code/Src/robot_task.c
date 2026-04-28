#include "robot_task.h"
#include "control.h"
#include "odometry.h"
#include "math.h"
#include "utils.h"

// Task Context Instance
static Robot_Context_t ctx;

// Speeds for different track sections
#define CRUISE_SPEED 0.3f    // m/s for straights and wavy lines
#define BOX_ENTRY_SPEED 0.1f // m/s when approaching 90-deg corners
#define TURN_SPEED 0.0f      // 0.0 means pivot-in-place for IMU turns

#define SEARCH_ANGLE (0.6f) // about 35 degree

// 记录全局的路口数量
static uint8_t junction_count = 0;
// 路口锁：防止在一个物理路口内，因为连续多帧视觉识别而重复决策
static bool is_in_junction = false;
static OpenMV_Possible_Direction_t locked_direction = Direction_NORMAL;

void RobotTask_Init(void)
{
    ctx.current_state = MISSION_IDLE;
    ctx.last_passed_marker = MARKER_START;
    Control_Init();
}

static float dynamic_throttling(float vision_error)
{
    float abs_error = fabs(vision_error);

    // Speed = Cruise_Speed - (Error * Drop_Factor)
    float target_speed = CRUISE_SPEED - (abs_error * 0.01f);

    if (target_speed < BOX_ENTRY_SPEED)
    {
        target_speed = BOX_ENTRY_SPEED;
    }
    return target_speed;
}

static uint8_t line_stable_count = 0;

void RobotTask_Update(OpenMV_Data_t *omv)
{
    // ---------------------------------------------------------
    // 1. FAULT DETECTION
    // ---------------------------------------------------------
    if (ctx.current_state == MISSION_FAULT_LOST_LINE)
    {
        // 只有看到 NORMAL 且 误差在可控范围内（比如线不在画面最边缘）
        if (omv->flag != 0xFF && fabs(omv->err_f) < 65.0f)
        {
            line_stable_count++;
            // 必须连续 3 帧（约 30-50ms）看到线，才认为恢复成功
            if (line_stable_count > 3)
            {
                ctx.current_state = MISSION_RUNNING;
                // 恢复瞬间使用较慢的慢速，给 PID 锁定的时间，防止甩尾
                Control_SetLineError(BOX_ENTRY_SPEED, omv->err_f);
                return;
            }
        }
        else
        {
            line_stable_count = 0;
        }
    }

    // ---------------------------------------------------------
    // MARKER TRACKING: Update Last Checkpoint
    // ---------------------------------------------------------
    // ctx.last_passed_marker = Marker_update(); TODO:full map

    //  ---------------------------------------------------------
    //  STATE MACHINE
    //  ---------------------------------------------------------
    switch (ctx.current_state)
    {
    case MISSION_IDLE:
    case MISSION_FINISHED:
        // Do nothing. Waiting for human button press to resume.
        Control_Stop();
        break;
    case MISSION_FAULT_LOST_LINE:
        Control_Stop(); // TODO: temp, should be deleted
        // --- 原地搜索状态机 ---
        // switch (ctx.search_step)
        // {
        // case 0: // 准备阶段：停止并等待惯性消失
        //     Control_Stop();
        //     if (fabs(Odometry_GetState()->angular_vel) < 0.2f)
        //     {
        //         ctx.search_step = 1;
        //         // 目标：向左看
        //         Control_SetIMUHeading(0.0f, Math_NormalizeAngle(ctx.search_base_yaw + SEARCH_ANGLE));
        //     }
        //     break;

        // case 1: // 正在向左看
        //     if (Control_IsHeadingSettled())
        //     {
        //         ctx.search_step = 2;
        //         // 目标：向右看
        //         Control_SetIMUHeading(0.0f, Math_NormalizeAngle(ctx.search_base_yaw - SEARCH_ANGLE));
        //     }
        //     break;

        // case 2: // 正在向右看
        //     if (Control_IsHeadingSettled())
        //     {
        //         ctx.search_step = 3;
        //         // 目标：回中
        //         Control_SetIMUHeading(0.0f, ctx.search_base_yaw);
        //     }
        //     break;

        // case 3: // 正在回中
        //     if (Control_IsHeadingSettled())
        //     {
        //         ctx.search_step = 4; // 搜索失败
        //         Control_Stop();
        //     }
        //     break;

        // case 4: // 彻底丢失
        //     // 此时电机完全停止，等待人工按键 AcknowledgePlacement
        //     Control_Stop();
        //     // TODO: Turn on RED LED, print to OLED:
        //     // "LOST! Move to Marker %d and Press Button", ctx.last_passed_marker
        //     break;
        // }
        break;

    case MISSION_RUNNING:
        // 1. 彻底丢线
        if (omv->flag == 0xFF) // TrackFlag.LOST
        {
            ctx.current_state = MISSION_FAULT_LOST_LINE;
            ctx.search_step = 0;
            ctx.search_base_yaw = Odometry_GetState()->theta;
            line_stable_count = 0;
            is_in_junction = false; // 重置路口锁
        }
        // 2. 遇到岔路口 (0x10 系列)
        else if ((omv->flag & 0xF0) == 0x10)
        {
            // 如果是刚进入这个路口，进行唯一一次路径决策
            if (!is_in_junction)
            {
                locked_direction = Decide_Shortest_Path(omv->flag, &junction_count);
                is_in_junction = true;
            }

            // 根据决策结果，挑选对应的误差喂给 PID
            float selected_error = 0.0f;
            switch (locked_direction)
            {
            case Direction_LEFT:
                selected_error = omv->err_l;
                break;
            case Direction_RIGHT:
                selected_error = omv->err_r;
                break;
            case Direction_FORWARD:
            case Direction_NORMAL:
            default:
                selected_error = omv->err_f;
                break;
            }

            // 使用视觉巡线 PID 顺着选定的线转弯
            // 建议转弯时速度稍降，防止由于视觉延迟冲出赛道
            Control_SetLineError(BOX_ENTRY_SPEED, selected_error);
        }
        // 3. 正常直线/单路弯道巡线 (NORMAL = 0x00)
        else
        {
            // 离开路口，解除锁定
            if (is_in_junction)
            {
                is_in_junction = false;
            }

            // 动态限速：误差越大速度越慢
            float target_speed = dynamic_throttling(omv->err_f);

            // 默认跟随 err_f
            Control_SetLineError(CRUISE_SPEED, omv->err_f);
        }
        break;
    }
}

void RobotTask_Start(void)
{
    ctx.current_state = MISSION_RUNNING;
    ctx.last_passed_marker = MARKER_START;
    Control_SetLineError(CRUISE_SPEED, 0.0f);
}

/**
 * @brief  Call this in your EXTI Interrupt Handler for the User Button
 *         It handles the rule: "restarted from the previous marker"
 */
void RobotTask_AcknowledgePlacement(void)
{
    if (ctx.current_state == MISSION_FAULT_LOST_LINE)
    {
        // 1. Get the baseline distance of the marker we were placed at
        float reset_distance = 0.0f;
        switch (ctx.last_passed_marker)
        {
        case MARKER_START:
            reset_distance = 0.0f;
            break;
        case MARKER_1_1:
            reset_distance = DIST_MARKER_1_1;
            break;
        case MARKER_1_2:
            reset_distance = DIST_MARKER_1_2;
            break;
        case MARKER_1_3:
            reset_distance = DIST_MARKER_1_3;
            break;
        case MARKER_1_4:
            reset_distance = DIST_MARKER_1_4;
            break;
        case MARKER_1_5:
            reset_distance = DIST_MARKER_1_5;
            break;
        }

        // 3. Clear Kinematic PID loops to prevent jerk
        Control_Init();

        Odometry_GetState()->distance = reset_distance;

        // 4. Resume Task
        ctx.current_state = MISSION_RUNNING;
        ctx.corner_1_3_cnt = 0;
        Control_SetLineError(CRUISE_SPEED, 0.0f);
    }
    else if (ctx.current_state == MISSION_IDLE)
    {
        RobotTask_Start();
    }
}