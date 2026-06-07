#include "robot_task.h"
#include "control.h"
#include "odometry.h"
#include <math.h>
#include <stdlib.h>
#include "utils.h"
#include "st7735.h"
#include "radar.h"

// Task Context Instance
Robot_Context_t ctx;

// Speeds for different track sections
#define CRUISE_SPEED 0.3f    // m/s for straights and wavy lines
#define BOX_ENTRY_SPEED 0.2f // m/s when approaching 90-deg corners
#define TURN_SPEED 0.2f      // 0.0 means pivot-in-place for IMU turns

#define SEARCH_ANGLE (0.6f) // about 35 degree

#define JUNC_WINDOW_SIZE 20
static Direction_t decision_buffer[JUNC_WINDOW_SIZE];
static uint8_t buffer_idx = 0;
static bool is_deciding = false;
static bool is_executing_junction = false;

static Direction_t chosen_direction = Direction_NORMAL;

/* ================================================================
 * Radar Pre-Scan (5 s at mission start / Key1)
 * ================================================================
 * Samples radar votes for PRE_SCAN_DURATION_MS.
 * If a valid LEFT/RIGHT direction is obtained, it is remembered
 * in ctx.pre_scan_dir and used later in the task-3 zone instead
 * of live radar voting — but still going through the deciding
 * window to keep timing consistent.
 * ================================================================ */

#define PRE_SCAN_DURATION_MS  5000U

static uint32_t pre_scan_start_tick = 0;
static bool     pre_scan_active     = false;

static void PreScan_Start(void)
{
    pre_scan_start_tick = HAL_GetTick();
    pre_scan_active     = true;
    ctx.pre_scan_valid  = false;
    ctx.pre_scan_dir    = Direction_NORMAL;
    Radar_Start();
}

static void PreScan_Update(void)
{
    if (!pre_scan_active)
        return;

    uint32_t now = HAL_GetTick();

    /* Check for early valid vote before timeout */
    Direction_t vote = Radar_GetAvoidanceDirection();

    if (vote == Direction_LEFT || vote == Direction_RIGHT)
    {
        /* Got a stable direction — record and stop radar */
        ctx.pre_scan_valid = true;
        ctx.pre_scan_dir   = vote;
        pre_scan_active    = false;
        Radar_Stop();
        return;
    }

    /* Timeout with no valid direction —
       leave radar running so the live-voting fallback path works */
    if ((now - pre_scan_start_tick) >= PRE_SCAN_DURATION_MS)
    {
        ctx.pre_scan_valid = false;
        ctx.pre_scan_dir   = Direction_NORMAL;
        pre_scan_active    = false;
        /* Radar keeps running — live Radar_GetAvoidanceDirection()
           will be used in the task-3 junction window */
    }
}

/* ================================================================ */

void RobotTask_Init(void)
{
    ctx.current_state = MISSION_IDLE;
    ctx.last_passed_marker = MARKER_START;
    ctx.task3_radar_done = false;
    Control_Init();
}

static float dynamic_throttling(float vision_error)
{
    float abs_error = fabs(vision_error);

    float speed_drop = (abs_error * abs_error) / 10000.0f * (CRUISE_SPEED - TURN_SPEED);

    float target_speed = CRUISE_SPEED - speed_drop;

    if (target_speed < TURN_SPEED)
    {
        target_speed = TURN_SPEED;
    }
    return target_speed;
}

Direction_t Get_Most_Frequent_Direction(Direction_t *buf, uint8_t size)
{
    int counts[3] = {0}; // 代表 RIGHT(-1), FORWARD(0), LEFT(1)
    for (uint8_t i = 0; i < size; i++)
    {
        counts[buf[i] + 1]++;
    }
    int max_idx = 1; // 默认 FORWARD
    if (counts[0] > counts[max_idx])
        max_idx = 0;
    if (counts[2] > counts[max_idx])
        max_idx = 2;
    return (Direction_t)(max_idx - 1);
}

void RobotTask_Start(void)
{
    is_deciding = false;
    is_executing_junction = false;
    buffer_idx = 0;
    ctx.current_state = MISSION_RUNNING;
    ctx.last_passed_marker = MARKER_START;
    Control_SetLineError(CRUISE_SPEED, 0.0f);

    /* Kick off 5 s radar pre-scan */
    PreScan_Start();
}

/**
 * @brief  Call this in your EXTI Interrupt Handler for the User Button
 *         It handles the rule: "restarted from the previous marker"
 */
void RobotTask_AcknowledgePlacement(void)
{
    if (ctx.current_state == MISSION_FAULT_LOST_LINE || ctx.current_state == MISSION_IDLE)
    {
        // 1. 获取该 Marker 的预设精准状态
        Marker_Info_t target = MAP_MARKERS[ctx.last_passed_marker];

        // 2. 彻底重置里程计到该 Marker 的物理位置
        Odometry_State_t *odo = Odometry_GetState();
        odo->x = target.x;
        odo->y = target.y;
        odo->distance = target.dist;

        // 4. 清理控制环路
        Control_Init();
        Radar_Stop(); // 重新开始，先停掉雷达

        // 5. 恢复运行
        ctx.current_state = MISSION_RUNNING;
        Control_SetLineError(CRUISE_SPEED, 0.0f);
    }
}

/**
 * @brief  Key1 (PC13) handler — relocate odometry to MARKER_1_4,
 *         enable task-3 radar zone, and start radar scanning.
 * @note   Called from EXTI callback (active-high button press).
 */
void RobotTask_TriggerTask3(void)
{
    /* 1. Relocate odometry to MARKER_1_4 preset position */
    Marker_Info_t target = MAP_MARKERS[MARKER_1_4];
    Odometry_State_t *odo = Odometry_GetState();
    odo->x = target.x;
    odo->y = target.y;
    odo->distance = target.dist;

    /* 2. Update mission context */
    ctx.last_passed_marker = MARKER_1_4;
    ctx.task3_radar_done = false;

    /* 3. Reset junction state machine */
    is_deciding = false;
    is_executing_junction = false;
    buffer_idx = 0;

    /* 4. Kick off 5 s radar pre-scan (replaces plain Radar_Start) */
    PreScan_Start();

    /* 5. Ensure we are in running state */
    if (ctx.current_state == MISSION_IDLE || ctx.current_state == MISSION_FAULT_LOST_LINE)
    {
        ctx.current_state = MISSION_RUNNING;
        Control_SetLineError(CRUISE_SPEED, 0.0f);
    }
}

static uint8_t line_stable_count = 0;

void RobotTask_Update(GraySensor_Data_t *gray)
{
    /* Tick the pre-scan state machine (5 s radar sampling) */
    PreScan_Update();

    // ---------------------------------------------------------
    // MARKER TRACKING: Update Last Checkpoint
    // ---------------------------------------------------------
    ctx.last_passed_marker = Marker_update();

    switch (ctx.current_state)
    {
    case MISSION_IDLE:
    case MISSION_FINISHED:
        // Do nothing. Waiting for human button press to resume.
        Control_Stop();
        break;
    case MISSION_FAULT_LOST_LINE:

        if (gray->flag != GraySensor_FLAG_LOST && abs(gray->err_f) < 95)
        {
            if (++line_stable_count > 3)
            {
                ctx.current_state = MISSION_RUNNING;
                Control_SetLineError(BOX_ENTRY_SPEED, gray->err_f);
                return;
            }
        }
        else
        {
            Control_Stop(); // TODO: temp, should be deleted
            line_stable_count = 0;
            // Execute_Line_Search_Sequence();
        }
        break;

    case MISSION_RUNNING:
    {
        bool is_in_task3_zone = (ctx.last_passed_marker == MARKER_1_4);
        static bool radar_running = false;
        // char buf[30];

        // 【新增】用于角度锁定的变量
        static float turn_start_yaw = 0.0f;
// 设定锁定角度门限：
// 这个角度既能避开十字路口干扰，又不会在波浪线过度转弯
#define JUNC_UNLOCK_ANGLE (PI / 3)

        static float last_junc_finish_dist = -1.0f;
#define JUNC_DIST_LIMIT 0.2f // 两次路口决策间的最小距离 (m)

        // 1. 彻底丢线 (最高优先级)
        if (gray->flag == GraySensor_FLAG_LOST)
        {
            ctx.current_state = MISSION_FAULT_LOST_LINE;
            ctx.search_step = 0;
            ctx.search_base_yaw = Odometry_GetState()->theta;
            line_stable_count = 0;
            is_deciding = false;
            is_executing_junction = false;
        }
        // 2. 基于“角度锁定”的转向执行逻辑 (★ 核心修改)
        else if (is_executing_junction)
        {
            // 计算当前转过的相对角度（考虑 -PI 到 PI 的翻转）
            float current_yaw = Odometry_GetState()->theta;
            float yaw_changed = fabs(Math_NormalizeAngle(current_yaw - turn_start_yaw));

            // 如果转角超过 40 度，或者传感器重新变回 NORMAL 状态且已经转了一定角度
            // 这样做可以兼容 90度直角弯 和 稍微缓一点的圆弧分支
            if (yaw_changed > JUNC_UNLOCK_ANGLE ||
                (yaw_changed > (PI / 6) && (gray->flag & 0xF0) == GraySensor_FLAG_NORMAL))
            {
                is_executing_junction = false;
                GraySensor_ForceSetLastErr(gray->err_f);
            }
            else
            {
                int16_t selected_error = 0;
                float current_speed = TURN_SPEED;

                // char buf[20];
                // sprintf(buf," %d ", chosen_direction);
                // ST7735_WriteString(5, 50, buf, ST7735_GREEN, ST7735_BLACK, 2);
                if (chosen_direction == Direction_LEFT)
                {
                    selected_error = (int16_t)gray->err_l * 1.5;
                    if (selected_error < -40)
                        GraySensor_ForceSetLastErr(selected_error);
                    else
                        GraySensor_ForceSetLastErr(-80);
                }
                else if (chosen_direction == Direction_RIGHT)
                {
                    selected_error = (int16_t)gray->err_r * 1.5;
                    if (selected_error > 40)
                        GraySensor_ForceSetLastErr(selected_error);
                    else
                        GraySensor_ForceSetLastErr(80);
                }
                else
                {
                    selected_error = gray->err_f;
                    current_speed = BOX_ENTRY_SPEED;
                    // Control_SetLineError(current_speed, selected_error);
                    // return;
                }
                // Control_SetIMUHeading(current_speed, Odometry_GetState()->theta + PI / 10);
                Control_SetLineError(current_speed, selected_error);
                return;
            }
        }

        float current_total_dist = Odometry_GetState()->distance;

        // 3. 遇到岔路口，且当前未在执行转向 -> 进入决策窗口期
        if (!is_executing_junction && ((gray->flag & 0xF0) == GraySensor_FLAG_JUNC))
        {
            if (current_total_dist - last_junc_finish_dist >= JUNC_DIST_LIMIT)
            {
                if (!is_deciding)
                {
                    is_deciding = true;
                    buffer_idx = 0;
                }

                float current_decide_speed = TURN_SPEED; // 默认路口减速

                if (is_in_task3_zone && !ctx.task3_radar_done)
                {
                    /* Pre-scan path: use remembered direction, still go
                       through the full deciding window for timing */
                    if (ctx.pre_scan_valid)
                    {
                        decision_buffer[buffer_idx++] = ctx.pre_scan_dir;
                    }
                    else
                    {
                        decision_buffer[buffer_idx++] = Radar_GetAvoidanceDirection();
                    }
                    current_decide_speed = 0.0f;
                }
                else
                {
                    decision_buffer[buffer_idx++] = Decide_Shortest_Path(gray->flag);
                }

                if (buffer_idx >= JUNC_WINDOW_SIZE)
                {
                    // 窗口填满，进行投票锁定
                    chosen_direction = Get_Most_Frequent_Direction(decision_buffer, JUNC_WINDOW_SIZE);
                    is_deciding = false;

                    if (chosen_direction == Direction_LEFT || chosen_direction == Direction_RIGHT)
                        is_executing_junction = true;

                    // ★ 记录转弯锁定的起始里程计距离
                    turn_start_yaw = Odometry_GetState()->theta;
                    last_junc_finish_dist = current_total_dist;

                    if (is_in_task3_zone)
                    {

                        Radar_Stop();
                        radar_running = false;
                        ctx.task3_radar_done = true;
                    }
                }
                // 决策窗口期内维持降速
                Control_SetLineError(current_decide_speed, gray->err_f);
            }
        }

        // 4. 正常直线/弯道/波浪线巡线 (NORMAL = 0x00)
        else if (!is_executing_junction)
        {
            // ★ 如果是在波浪线处发生的短时误判，由于 flag 变回 NORMAL，
            // 这里的 is_deciding = false 会自动清空决策缓存，起到了完美抗干扰滤波的作用！
            is_deciding = false;

            float dynamic_speed = dynamic_throttling(gray->err_f);
            Control_SetLineError(dynamic_speed, gray->err_f);
        }

        break;
    }
    }
}
