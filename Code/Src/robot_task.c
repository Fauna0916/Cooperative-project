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

// Kinematic Constants
#define WHEELBASE_OFFSET 0.185f // Distance from camera view center to wheel axis (m)

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

void RobotTask_Update(OpenMV_Flag_t vision_flag, float vision_error)
{
    // ---------------------------------------------------------
    // 1. FAULT DETECTION
    // ---------------------------------------------------------
    if (ctx.current_state == MISSION_FAULT_LOST_LINE)
    {
        // 只有看到 NORMAL 且 误差在可控范围内（比如线不在画面最边缘）
        if (vision_flag == OpenMV_FLAG_NORMAL && fabs(vision_error) < 65.0f)
        {
            line_stable_count++;
            // 必须连续 3 帧（约 30-50ms）看到线，才认为恢复成功
            if (line_stable_count > 3)
            {
                ctx.current_state = MISSION_RUNNING;
                // 恢复瞬间使用较慢的慢速，给 PID 锁定的时间，防止甩尾
                Control_SetLineError(BOX_ENTRY_SPEED, vision_error);
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
        if (vision_flag == OpenMV_FLAG_LOST)
        {
            ctx.current_state = MISSION_FAULT_LOST_LINE;
            ctx.search_step = 0;
            ctx.search_base_yaw = Odometry_GetState()->theta;
            line_stable_count = 0;
        }
        // --- Normal Line Following ---
        // Detect upcoming corners
        else if (vision_flag == OpenMV_FLAG_CORNER_LEFT || vision_flag == OpenMV_FLAG_CORNER_RIGHT)
        { 
            if (ctx.last_passed_marker == MARKER_1_3)
            {
                ctx.corner_1_3_cnt = 0;
                ctx.corner_1_3_cnt++;
                if (ctx.corner_1_3_cnt == 1)
                {
                    Control_SetLineError(CRUISE_SPEED, vision_error);
                }
                else
                {
                    ctx.corner_1_3_cnt = 0;
                    // Query the Distance-Gated Map
                    Turn_Direction_t map_dir = Get_Distance_Gated_Turn(vision_flag, ctx.last_passed_marker);

                    // 1. Trust the Map over the OpenMV
                    float target_yaw = Odometry_GetState()->theta + (map_dir * (PI / 2.0f)); // TODO integrat map_dir and openmv flag
                    // 2. Setup corner sequence
                    ctx.target_corner_yaw = target_yaw;

                    ctx.corner_step = 0;
                    ctx.current_state = MISSION_CORNERING;
                }
            }
            else
            {
                // Query the Distance-Gated Map
                Turn_Direction_t map_dir = Get_Distance_Gated_Turn(vision_flag, ctx.last_passed_marker);

                if (map_dir != TURN_NONE)
                {
                    // 1. Trust the Map over the OpenMV
                    float target_yaw = Odometry_GetState()->theta + (map_dir * (PI / 2.0f)); // TODO integrat map_dir and openmv flag
                    // 2. Setup corner sequence
                    ctx.target_corner_yaw = target_yaw;

                    ctx.corner_step = 0;
                    ctx.current_state = MISSION_CORNERING;
                }
                else
                {
                    // OpenMV saw a corner, but map says no! Ignore it and keep following line.
                    Control_SetLineError(CRUISE_SPEED, vision_error);
                }
            }
        }
        else
        {

            // Determine speed based on error (Dynamic Throttling)
            float target_speed = dynamic_throttling(vision_error);

            Control_SetLineError(target_speed, vision_error);
            // TODO: Odometry-based Marker Updating
            // We need to track distance traveled to know when we pass 1.1, 1.2, etc.
            // Example: if(Odometry_GetDistance() > 2.5f) ctx.last_passed_marker = MARKER_1_2;
        }
        break;

    case MISSION_CORNERING:
        // --- 90-Degree Box Turn Sequence ---

        // Step 0: The camera sees the corner early. The wheels are not on the intersection yet.
        // We must drive forward slightly (blindly) so the wheels align with the corner.
        if (ctx.corner_step == 0)
        {
            static float start_dist = 0;
            if (start_dist == 0)
                start_dist = Odometry_GetState()->distance;

            Control_SetVelocity(BOX_ENTRY_SPEED, 0.0f); // Drive straight blind

            if (Odometry_GetState()->distance - start_dist >= WHEELBASE_OFFSET)
            {
                start_dist = 0;
                ctx.corner_step = 1;

                // Trigger the IMU Turn
                Control_SetIMUHeading(TURN_SPEED, ctx.target_corner_yaw);
            }
        }
        // Step 1: Wait for IMU PID to settle
        else if (ctx.corner_step == 1)
        {
            if (Control_IsHeadingSettled())
            {
                // Turn complete. Resume Vision Tracking
                ctx.current_state = MISSION_RUNNING;
                Control_SetLineError(CRUISE_SPEED, vision_error); // Reset OpenMV PID // TODO
            }
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