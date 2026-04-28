#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "openmv.h"
#include "control.h"
#include "map.h"

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_RUNNING,
    MISSION_CORNERING,       // Executing a 90-deg box turn
    MISSION_FAULT_LOST_LINE, // Waiting for human placement
    MISSION_FINISHED
} Mission_State_t;

typedef struct
{
    Mission_State_t current_state;
    Track_Marker_t last_passed_marker; // The fallback point

    // Variables for 90-deg cornering sequence
    float target_corner_yaw;
    uint32_t corner_start_time;
    uint8_t corner_step; // 0 = forward offset, 1 = IMU turn

    uint8_t corner_1_3_cnt;

    // search
    uint8_t search_step;   // 0:停止记录, 1:左转, 2:右转, 3:回中, 4:彻底放弃
    float search_base_yaw; // 丢失线时的基础航向
} Robot_Context_t;

void RobotTask_Init(void);
void RobotTask_Start(void);
void RobotTask_Update(OpenMV_Flag_t vision_flag, float vision_error);
void RobotTask_AcknowledgePlacement(void); // Called by EXTI User Button

#endif