#include "map.h"
#include "gray_sensor.h"
#include "utils.h"
#include "st7735.h"

#define X_MOVE_WEIGHT 0.30f // X轴移动权重 30cm
#define Y_MOVE_WEIGHT 0.70f // Y轴移动权重 50cm

float Predict_Target_Theta(float current_theta, Direction_t action)
{
    if (action == Direction_FORWARD)
        return current_theta;
    if (action == Direction_LEFT)
        return Math_NormalizeAngle(current_theta + PI / 2);
    if (action == Direction_RIGHT)
        return Math_NormalizeAngle(current_theta - PI / 2);
    return current_theta;
}

Direction_t Decide_Shortest_Path(uint8_t junction_flag)
{
    // 如果不是路口，直接返回 NORMAL
    if ((junction_flag & 0xF0) != GraySensor_FLAG_JUNC)
    {
        return Direction_NORMAL;
    }

    // 解析当前路口有哪些可行方向
    Direction_t available_dirs[3];
    uint8_t dir_count = 0;

    if (junction_flag & 0x01)
        available_dirs[dir_count++] = Direction_LEFT;
    if (junction_flag & 0x02)
        available_dirs[dir_count++] = Direction_FORWARD;
    if (junction_flag & 0x04)
        available_dirs[dir_count++] = Direction_RIGHT;      

    // 如果解码错误，默认往前走
    if (dir_count == 0)
        return Direction_FORWARD;

    float sign_x = (Odometry_GetState()->x >= 0) ? 1.0f : -1.0f;

    //printf("x:%.1f,%.1f\r\n", Odometry_GetState()->x, Odometry_GetState()->y);

    Direction_t best_choice = available_dirs[0];

    float max_score = -999.0f;

    char buf[40];
    sprintf(buf, "theta:%.2f", Odometry_GetState()->theta * 57.29578f);
    ST7735_WriteString(2, 50, buf, ST7735_WHITE, ST7735_BLACK, 1);

    for (uint8_t i = 0; i < dir_count; i++)
    {
        float pred_theta = Predict_Target_Theta(Odometry_GetState()->theta, available_dirs[i]);

        // 2. 计算该朝向在 X/Y 上的单位分量
        float vx = -sinf(pred_theta);
        float vy = cosf(pred_theta);

        // 3. 计算增长增益
        float gain = (vx * sign_x * X_MOVE_WEIGHT) + (vy * Y_MOVE_WEIGHT);

        if (gain > max_score)
        {
            max_score = gain;
            best_choice = available_dirs[i];
        }

        char buf[40];
        sprintf(buf, "D%d,G:%.1f,x:%.1f,y:%.1f", available_dirs[i], gain, vx, vy);
        ST7735_WriteString(2, (i+1)*10, buf, ST7735_WHITE, ST7735_BLACK, 1);
        HAL_Delay(10);
        printf("ava:%d,%f\r\n", available_dirs[i], gain);
    }

    switch (best_choice)
    {
    case Direction_RIGHT:
        ST7735_WriteString(2, 65, "RIGHT", ST7735_WHITE, ST7735_BLACK, 1);
        break;
    case Direction_FORWARD:
        ST7735_WriteString(2, 65, "FORW", ST7735_WHITE, ST7735_BLACK, 1);
        break;
    case Direction_LEFT:
        ST7735_WriteString(2, 65, "LEFT", ST7735_WHITE, ST7735_BLACK, 1);
        break;
    case Direction_NORMAL:
        ST7735_WriteString(2, 65, "NORMAL", ST7735_WHITE, ST7735_BLACK, 1);
        break;
    }
    //printf("chosen:%d\r\n", best_choice);
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