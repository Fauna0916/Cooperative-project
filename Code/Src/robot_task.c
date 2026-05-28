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
#define TURN_SPEED 0.05f     // 0.0 means pivot-in-place for IMU turns

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
    Control_Init();
}

static float dynamic_throttling(float vision_error)
{
    float abs_error = fabs(vision_error);

    float speed_drop = (abs_error * abs_error) / 10000.0f * (CRUISE_SPEED - BOX_ENTRY_SPEED);

    float target_speed = CRUISE_SPEED - speed_drop;

    if (target_speed < BOX_ENTRY_SPEED)
    {
        target_speed = BOX_ENTRY_SPEED;
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

void dir_display(Direction_t dir)
{
    static uint8_t cnt = 0;
    cnt++;
    switch (dir)
    {
    case Direction_RIGHT:
        printf("%d, R\r\n",cnt);
        break;
    case Direction_FORWARD:
        printf("%d, F\r\n", cnt);
        break;
    case Direction_LEFT:
        printf("%d, L\r\n", cnt);
        break;
    case Direction_NORMAL:
        printf("%d, NORM\r\n", cnt);
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
        else if ((gray->flag & 0xF0) == GraySensor_FLAG_JUNC)
        {

            // --- 阶段 A: 进入决策窗口 ---
            if (!is_executing_junction && !is_deciding)
            {
                is_deciding = true;
                buffer_idx = 0;
            }

            if (is_deciding)
            {
                // 在窗口期内持续计算但不执行大动作
                decision_buffer[buffer_idx++] = Decide_Shortest_Path(gray->flag);

                if (buffer_idx >= JUNC_WINDOW_SIZE)
                {
                    // 窗口填满，进行投票
                    chosen_direction = Get_Most_Frequent_Direction(decision_buffer, JUNC_WINDOW_SIZE);
                    dir_display(chosen_direction); // TODO: debug
                    // Control_Stop();
                    // HAL_Delay(2000);
                    is_deciding = false;
                    is_executing_junction = true;
                }
                // 窗口期内先维持原速直行或微减速
                Control_SetLineError(BOX_ENTRY_SPEED, gray->err_f);
                return;
            }

            // --- 阶段 B: 执行锁定后的决策 ---
            if (is_executing_junction)
            {
                int16_t selected_error = 0;
                float current_speed = TURN_SPEED;

                if (chosen_direction == Direction_LEFT)
                    selected_error = gray->err_l;
                else if (chosen_direction == Direction_RIGHT)
                    selected_error = gray->err_r;
                else
                {
                    selected_error = gray->err_f;
                    current_speed = BOX_ENTRY_SPEED;
                }

                // 【自纠错机制】：如果选了转弯，但在该方向上完全看不到线 (假设偏差处于极端值)
                if (abs(selected_error) > 95 && (gray->raw_data & 0x18))
                { // 0x18 是中间两个灯
                    // 可能是误判了，中间线其实很稳。解除锁定，切换回直行
                    is_executing_junction = false;
                    return;
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