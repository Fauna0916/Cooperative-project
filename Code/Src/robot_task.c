#include "robot_task.h"
#include "control.h"
#include "odometry.h"
#include <math.h>
#include <stdlib.h>
#include "utils.h"

#define WHEELBASE_OFFSET 0.185f // Distance from camera view center to wheel axis (m)
#define BACKWARD_OFFSET 0.1f

// Task Context Instance
static Robot_Context_t ctx;

// Speeds for different track sections
#define CRUISE_SPEED 0.3f    // m/s for straights and wavy lines
#define BOX_ENTRY_SPEED 0.1f // m/s when approaching 90-deg corners
#define TURN_SPEED 0.0f      // 0.0 means pivot-in-place for IMU turns

#define SEARCH_ANGLE (0.6f) // about 35 degree

static Direction_t chosen_direction = Direction_NORMAL;

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
    float target_speed = CRUISE_SPEED - (abs_error * 0.20f);

    if (target_speed < BOX_ENTRY_SPEED)
    {
        target_speed = BOX_ENTRY_SPEED;
    }
    return target_speed;
}

/**
 * @brief  原地扫视搜索黑线状态机
 * @note   由 RobotTask_Update 在 MISSION_FAULT_LOST_LINE 状态下调用
 */
void Execute_Line_Search_Sequence(void)
{
    Odometry_State_t *odo = Odometry_GetState();

    switch (ctx.search_step)
    {
    case 0:
        Control_Stop();
        if (fabs(odo->angular_vel) < 0.2f)
        {
            ctx.search_step = 1;
            // 开始向左扫视：基础航向 + 搜索角
            float target_yaw = Math_NormalizeAngle(ctx.search_base_yaw + SEARCH_ANGLE);
            Control_SetIMUHeading(0.0f, target_yaw);
        }
        break;

    case 1: // --- 正在向左扫视 ---
        if (Control_IsHeadingSettled())
        {
            ctx.search_step = 2;
            // 转向右侧扫视：基础航向 - 搜索角
            float target_yaw = Math_NormalizeAngle(ctx.search_base_yaw - SEARCH_ANGLE);
            Control_SetIMUHeading(0.0f, target_yaw);
        }
        break;

    case 2: // --- 正在向右扫视 ---
        if (Control_IsHeadingSettled())
        {
            ctx.search_step = 3;
            // 扫视一圈没发现，回到最初丢失的方向，等待人工救援
            Control_SetIMUHeading(0.0f, ctx.search_base_yaw);
        }
        break;

    case 3: // --- 正在回正中心 ---
        if (Control_IsHeadingSettled())
        {
            ctx.search_step = 4; // 搜索失败，进入彻底丢失模式
        }
        break;

    case 4: // --- 彻底丢失阶段 ---
        Control_Stop();
        break;

    default:
        ctx.search_step = 4;
        break;
    }
}

static uint8_t line_stable_count = 0;

void RobotTask_Update(GraySensor_Data_t *gray)
{

    // ---------------------------------------------------------
    // MARKER TRACKING: Update Last Checkpoint
    // ---------------------------------------------------------
    // ctx.last_passed_marker = Marker_update(); TODO:full map

    switch (ctx.current_state)
    {
    case MISSION_IDLE:
    case MISSION_FINISHED:
        // Do nothing. Waiting for human button press to resume.
        Control_Stop();
        break;
    case MISSION_FAULT_LOST_LINE:
        // Control_Stop(); // TODO: temp, should be deleted

        if (gray->flag != GraySensor_FLAG_LOST && abs(gray->err_f) < 75)
        {
            if (++line_stable_count > 2)
            {
                ctx.current_state = MISSION_RUNNING;
                Control_SetLineError(BOX_ENTRY_SPEED, gray->err_f);
                return;
            }
        }
        else
        {
            line_stable_count = 0;
            Execute_Line_Search_Sequence();
        }
        break;

    case MISSION_RUNNING:
        // 1. 彻底丢线
        if (gray->flag == GraySensor_FLAG_LOST) // TrackFlag.LOST
        {
            ctx.current_state = MISSION_FAULT_LOST_LINE;
            ctx.search_step = 0;
            ctx.search_base_yaw = Odometry_GetState()->theta;
            line_stable_count = 0;
        }
        // 2. 遇到岔路口 (0x10 系列)
        else if ((gray->flag & 0xF0) == GraySensor_FLAG_JUNC)
        {

            chosen_direction = Decide_Shortest_Path(gray->flag);

            int16_t selected_error = 0.0f;
            switch (chosen_direction)
            {
            case Direction_LEFT:
                selected_error = gray->err_l;
                break;
            case Direction_RIGHT:
                selected_error = gray->err_r;
                break;
            case Direction_FORWARD:
            case Direction_NORMAL:
            default:
                selected_error = gray->err_f;
                break;
            }
            if (chosen_direction == Direction_LEFT || chosen_direction == Direction_RIGHT)
            {
                Control_SetLineError(TURN_SPEED, selected_error);
            }
            else
            {
                float dynamic_speed = dynamic_throttling(selected_error);
                Control_SetLineError(dynamic_speed, selected_error);
            }
        }
        // 3. 正常直线/单路弯道巡线 (NORMAL = 0x00)
        else
        {
            float dynamic_speed = dynamic_throttling(gray->err_f);
            Control_SetLineError(dynamic_speed, gray->err_f);
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