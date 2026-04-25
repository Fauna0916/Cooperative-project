#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "openmv.h"
#include "control.h"

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_RUNNING,
    MISSION_CORNERING,       // Executing a 90-deg box turn
    MISSION_FAULT_LOST_LINE, // Waiting for human placement
    MISSION_FINISHED
} Mission_State_t;

// --- Track Markers (Checkpoints) ---
typedef enum
{
    MARKER_START = 0,
    MARKER_1_1,
    MARKER_1_2,
    MARKER_1_3,
    MARKER_1_4,
    MARKER_1_5
} Track_Marker_t;

// --- Task Context Structure ---
typedef struct
{
    Mission_State_t current_state;
    Track_Marker_t last_passed_marker; // The fallback point

    // Variables for 90-deg cornering sequence
    float target_corner_yaw;
    uint32_t corner_start_time;
    uint8_t corner_step; // 0 = forward offset, 1 = IMU turn

} Robot_Context_t;

void RobotTask_Init(void);
void RobotTask_Start(void);
void RobotTask_Update(OpenMV_Flag_t vision_flag, float vision_error);
void RobotTask_AcknowledgePlacement(void); // Called by EXTI User Button

#endif