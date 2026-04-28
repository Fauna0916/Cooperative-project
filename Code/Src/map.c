#include "map.h"
#include "utils.h"
#include "odometry.h"


float Predict_Target_Theta(float current_theta, OpenMV_Possible_Direction_t action)
{
    if (action == Direction_FORWARD)
        return current_theta;
    if (action == Direction_LEFT)
        return Math_NormalizeAngle(current_theta + PI/2);
    if (action == Direction_RIGHT)
        return Math_NormalizeAngle(current_theta - PI / 2);
    return current_theta;
}

OpenMV_Possible_Direction_t Decide_Shortest_Path(uint8_t junction_flag, uint8_t* junction_count)
{
    // 如果不是路口，直接返回 NORMAL
    if ((junction_flag & 0xF0) != 0x10)
    {
        return Direction_NORMAL;
    }

    // 解析当前路口有哪些可行方向
    OpenMV_Possible_Direction_t available_dirs[3];
    uint8_t dir_count = 0;

    if (junction_flag & 0x01) available_dirs[dir_count++] = Direction_LEFT;
    if (junction_flag & 0x02) available_dirs[dir_count++] = Direction_FORWARD;
    if (junction_flag & 0x04) available_dirs[dir_count++] = Direction_RIGHT;

    // 如果解码错误，默认往前走
    if (dir_count == 0) return Direction_FORWARD;

    // 进入新路口，计数器+1
    (*junction_count)++;
    OpenMV_Possible_Direction_t best_choice = available_dirs[0];

    // --- 决策逻辑 1：第一个路口 (1.2点处寻求“归心”，最大化 X 轴移动) ---
    if ((*junction_count) == 1)
    {
        float max_x_impact = -1.0f;

        for (uint8_t i = 0; i < dir_count; i++)
        {
            float target_theta = Predict_Target_Theta(Odometry_GetState()->theta, available_dirs[i]);
            // 计算该方向产生的 X 轴位移趋势 (cos)，期望远离起点 x=0
            float x_trend = fabsf(cosf(target_theta));

            if (x_trend > max_x_impact)
            {
                max_x_impact = x_trend;
                best_choice = available_dirs[i];
            }
        }
    }
    // --- 决策逻辑 2：后续路口 (1.3点及以后寻求“向上”，最大化 Y 轴移动) ---
    else
    {
        float max_y_trend = -2.0f; 

        for (uint8_t i = 0; i < dir_count; i++)
        {
            float target_theta = Predict_Target_Theta(Odometry_GetState()->theta, available_dirs[i]);
            float y_trend = sinf(target_theta); // sin代表Y轴分量

            // 目标是让 y 增加的方向 (North)
            if (y_trend > max_y_trend)
            {
                max_y_trend = y_trend;
                best_choice = available_dirs[i];
            }
        }
    }

    return best_choice;
}


Track_Marker_t Marker_update(void)
{
    float current_dist = Odometry_GetState()->distance;

    if (current_dist >= DIST_MARKER_1_5)
        return MARKER_1_5;
    else if (current_dist >= DIST_MARKER_1_4)
        return MARKER_1_4;
    else if (current_dist >= DIST_MARKER_1_3)
        return MARKER_1_3;
    else if (current_dist >= DIST_MARKER_1_2)
        return MARKER_1_2;
    else if (current_dist >= DIST_MARKER_1_1)
        return MARKER_1_1;
    else
        return MARKER_START;
}

// /**
//  * @brief Uses odometry distance to force correct turn direction, ignoring OpenMV noise.
//  */
// Turn_Direction_t Get_Distance_Gated_Turn(OpenMV_Flag_t dir, Track_Marker_t current_marker)
// {
//     float current_dist = Odometry_GetState()->distance;

//     // if (current_marker < MARKER_1_2)
//     // {
//     //     return TURN_NONE;
//     // }
//     // else if (current_marker == MARKER_1_3)
//     // {
//     // }
//     // else if (current_marker == MARKER_1_4)
//     // {
//     // }
//     // else
//     // {
//     //     if (dir == OpenMV_FLAG_CORNER_LEFT)
//     //         return TURN_LEFT;
//     //     else
//     //         return TURN_RIGHT;
//     // }

//     if (dir == OpenMV_FLAG_CORNER_LEFT)
//         return TURN_LEFT;
//     else
//         return TURN_RIGHT;

//     // // --- Box Sequence between Marker 1.2 and 1.3 ---
//     // // Example: 1st Box turn at 4.8m -> LEFT
//     // if (current_dist >= 10.0f && current_dist <= 5.0f)
//     //     return TURN_LEFT;

//     // // Example: 2nd Box turn at 5.5m -> RIGHT
//     // if (current_dist >= 5.3f && current_dist <= 5.7f)
//     //     return TURN_RIGHT;

//     // // Example: 3rd Box turn at 6.2m -> LEFT
//     // if (current_dist >= 6.0f && current_dist <= 6.4f)
//     //     return TURN_LEFT;

// }