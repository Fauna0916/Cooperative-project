#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "gray_sensor.h"
#include "control.h"
#include "map.h"
#include "stdbool.h"

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_RUNNING,
    MISSION_FAULT_LOST_LINE, // Waiting for human placement
    MISSION_FINISHED
} Mission_State_t;

typedef struct
{
    Mission_State_t current_state;
    Track_Marker_t last_passed_marker; // The fallback point
    bool task3_radar_done;

    /* Pre-scan: 5 s radar sampling at mission start / Key1 */
    Direction_t pre_scan_dir;   // remembered obstacle side from pre-scan
    bool        pre_scan_valid; // true if pre-scan produced a valid direction

    // search
    uint8_t search_step;   // 0:停止记录, 1:左转, 2:右转, 3:回中, 4:彻底放弃
    float search_base_yaw; // 丢失线时的基础航向
} Robot_Context_t;

void RobotTask_Init(void);
void RobotTask_Start(void);
void RobotTask_Update(GraySensor_Data_t *gray);
void RobotTask_AcknowledgePlacement(void); // Called by EXTI User Button
void RobotTask_TriggerTask3(void);         // Key1: relocate to MARKER_1_4 + radar scan

#endif