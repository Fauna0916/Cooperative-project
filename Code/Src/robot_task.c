#include "robot_task.h"
#include "control.h"
#include "odometry.h"
#include <math.h>
#include <stdlib.h>
#include "utils.h"
#include "st7735.h"
#include "radar.h"

#define WHEELBASE_OFFSET 0.185f // Distance from camera view center to wheel axis (m)
#define BACKWARD_OFFSET 0.1f

// Task Context Instance
Robot_Context_t ctx;

// Speeds for different track sections
#define CRUISE_SPEED 0.3f    // m/s for straights and wavy lines
#define BOX_ENTRY_SPEED 0.2f // m/s when approaching 90-deg corners
#define TURN_SPEED 0.15f     // 0.0 means pivot-in-place for IMU turns

#define SEARCH_ANGLE (0.6f) // about 35 degree

#define JUNC_WINDOW_SIZE 20
static Direction_t decision_buffer[JUNC_WINDOW_SIZE];
static uint8_t buffer_idx = 0;
static bool is_deciding = false;
static bool is_executing_junction = false;

static Direction_t chosen_direction = Direction_NORMAL;

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

void RobotTask_Start(void)
{
    is_deciding = false;
    is_executing_junction = false;
    buffer_idx = 0;
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

static uint8_t line_stable_count = 0;

void RobotTask_Update(GraySensor_Data_t *gray)
{

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
        // Control_Stop(); // TODO: temp, should be deleted

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
            line_stable_count = 0;
            // Execute_Line_Search_Sequence();
        }
        break;

    case MISSION_RUNNING:
    {
        bool is_in_task3_zone = (ctx.last_passed_marker == MARKER_1_4);
        static bool radar_running = false;
        char buf[30];

        if (is_in_task3_zone && !radar_running && !ctx.task3_radar_done)
        {
            Radar_Start();
            radar_running = true;
            sprintf(buf, "Radae Start");
            ST7735_WriteString(5, 2, buf, ST7735_GREEN, ST7735_BLACK, 2);
        }

        // 1. 彻底丢线
        if (gray->flag == GraySensor_FLAG_LOST) // TrackFlag.LOST
        {
            ctx.current_state = MISSION_FAULT_LOST_LINE;
            ctx.search_step = 0;
            ctx.search_base_yaw = Odometry_GetState()->theta;
            line_stable_count = 0;
            is_deciding = false;
            is_executing_junction = false;
        }
        // 2. 遇到岔路口 (0x10 系列)
        else if ((gray->flag & 0xF0) == GraySensor_FLAG_JUNC || is_executing_junction)
        {

            // --- 阶段 A: 进入决策窗口 ---
            if (!is_executing_junction && !is_deciding)
            {
                is_deciding = true;
                buffer_idx = 0;
            }

            if (is_deciding)
            {
                float current_decide_speed = TURN_SPEED; // 默认路口减速

                if (is_in_task3_zone && !ctx.task3_radar_done)
                {
                    decision_buffer[buffer_idx++] = Radar_GetAvoidanceDirection();
                    current_decide_speed = 0.0f;

                    sprintf(buf, "Radae Deciding");
                    ST7735_WriteString(5, 2, buf, ST7735_GREEN, ST7735_BLACK, 2);
                }
                else
                {
                    decision_buffer[buffer_idx++] = Decide_Shortest_Path(gray->flag);
                    current_decide_speed = TURN_SPEED;
                }

                if (buffer_idx >= JUNC_WINDOW_SIZE)
                {
                    // 窗口填满，进行投票
                    chosen_direction = Get_Most_Frequent_Direction(decision_buffer, JUNC_WINDOW_SIZE);
                    is_deciding = false;
                    is_executing_junction = true;

                    ctx.is_target_south = false;

                    if (is_in_task3_zone)
                    {
                        Radar_Stop();
                        radar_running = false;
                        ctx.task3_radar_done = true;
                        sprintf(buf, "Radae Stop");
                        ST7735_WriteString(5, 2, buf, ST7735_GREEN, ST7735_BLACK, 2);

                        ctx.is_target_south = true;
                    }
                }
                // 窗口期内先维持原速直行或微减速
                Control_SetLineError(current_decide_speed, gray->err_f);
                return;
            }
            // --- 阶段 B: 执行锁定后的决策 ---
            if (is_executing_junction)
            {
                int16_t selected_error = 0;
                float current_speed = TURN_SPEED;

                if (chosen_direction == Direction_LEFT)
                {
                    selected_error = gray->err_l;

                    if (selected_error < -40)
                        GraySensor_ForceSetLastErr(selected_error);
                    else
                        GraySensor_ForceSetLastErr(-80);
                }
                else if (chosen_direction == Direction_RIGHT)
                {
                    selected_error = gray->err_r;
                    if (selected_error > 40)
                        GraySensor_ForceSetLastErr(selected_error);
                    else
                        GraySensor_ForceSetLastErr(80);
                }
                else
                {
                    selected_error = gray->err_f;
                    current_speed = BOX_ENTRY_SPEED;
                }

                Control_SetLineError(current_speed, selected_error);
            }
        }
        // 3. 正常直线/单路弯道巡线 (NORMAL = 0x00)
        else
        {
            is_deciding = false;
            is_executing_junction = false;
            float dynamic_speed = dynamic_throttling(gray->err_f);
            Control_SetLineError(dynamic_speed, gray->err_f);
        }
        break;
    }
    }
}
