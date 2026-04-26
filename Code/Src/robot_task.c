#include "robot_task.h"
#include "control.h"
#include "odometry.h"
#include "math.h"
#include "stdio.h"

// Task Context Instance
static Robot_Context_t ctx;

// Speeds for different track sections
#define CRUISE_SPEED 0.3f    // m/s for straights and wavy lines
#define BOX_ENTRY_SPEED 0.1f // m/s when approaching 90-deg corners
#define TURN_SPEED 0.0f      // 0.0 means pivot-in-place for IMU turns

// Kinematic Constants
#define WHEELBASE_OFFSET 0.20f // Distance from camera view center to wheel axis (m)

void RobotTask_Init(void)
{
    ctx.current_state = MISSION_IDLE;
    ctx.last_passed_marker = MARKER_START;
    Control_Init();
}

void RobotTask_Update(OpenMV_Flag_t vision_flag, float vision_error)
{
    // ---------------------------------------------------------
    // 1. FAULT DETECTION (Top Priority)
    // ---------------------------------------------------------
    if (vision_flag == OpenMV_FLAG_LOST && ctx.current_state == MISSION_RUNNING)
    {
        // 1. Instantly kill motors to prevent driving off the mat
        Control_Stop();

        // 2. Transition to fault state
        ctx.current_state = MISSION_FAULT_LOST_LINE;

        // TODO: Turn on RED LED, print to OLED:
        // "LOST! Move to Marker %d and Press Button", ctx.last_passed_marker
        return;
    }

    // recover
    if (vision_flag != OpenMV_FLAG_LOST && ctx.current_state == MISSION_FAULT_LOST_LINE)
    {
        ctx.current_state = MISSION_RUNNING;
    }

    //printf("state:%d\r\n", ctx.current_state);
    // ---------------------------------------------------------
    // 2. STATE MACHINE
    // ---------------------------------------------------------
    switch (ctx.current_state)
    {
    case MISSION_IDLE:
    case MISSION_FAULT_LOST_LINE:
    case MISSION_FINISHED:
        // Do nothing. Waiting for human button press to resume.
        Control_Stop();
        break;

    case MISSION_RUNNING:
        // --- Normal Line Following ---
        // Detect upcoming corners
        if (vision_flag == OpenMV_FLAG_CORNER_LEFT || vision_flag == OpenMV_FLAG_CORNER_RIGHT)
        {
            ctx.current_state = MISSION_CORNERING;
            ctx.corner_step = 0; // Reset corner sequence

            // Calculate target Yaw mathematically based on current yaw
            float current_yaw = Odometry_GetState()->theta;
            float turn_angle = (vision_flag == OpenMV_FLAG_CORNER_LEFT) ? (PI / 2.0f) : (-PI / 2.0f);
            ctx.target_corner_yaw = current_yaw + turn_angle;
        }
        else
        {

            // Determine speed based on error (Dynamic Throttling)
            float target_speed = CRUISE_SPEED;
            if (fabs(vision_error) > 25.0f)
            {
                target_speed = BOX_ENTRY_SPEED; // Slow down for heavy bends
            }

            // Pass to your kinematics layer
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
        // 1. Reset Odometry so kinematics don't jump
        Odometry_Init(0.0f, 0.0f, 0.0f);

        // 2. Clear Kinematic PID loops
        Control_Init();

        // 3. Resume Task (Assume human placed it perfectly on the line at the marker)
        ctx.current_state = MISSION_RUNNING;
        Control_SetLineError(CRUISE_SPEED, 0.0f);
    }
    else if (ctx.current_state == MISSION_IDLE)
    {
        RobotTask_Start();
    }
}