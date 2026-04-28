#include "map.h"

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
}

/**
 * @brief Uses odometry distance to force correct turn direction, ignoring OpenMV noise.
 */
Turn_Direction_t Get_Distance_Gated_Turn(OpenMV_Flag_t dir, Track_Marker_t current_marker)
{
    float current_dist = Odometry_GetState()->distance;

    // if (current_marker < MARKER_1_2)
    // {
    //     return TURN_NONE;
    // }
    // else if (current_marker == MARKER_1_3)
    // {
    // }
    // else if (current_marker == MARKER_1_4)
    // {
    // }
    // else
    // {
    //     if (dir == OpenMV_FLAG_CORNER_LEFT)
    //         return TURN_LEFT;
    //     else
    //         return TURN_RIGHT;
    // }

    if (dir == OpenMV_FLAG_CORNER_LEFT)
        return TURN_LEFT;
    else
        return TURN_RIGHT;

    // // --- Box Sequence between Marker 1.2 and 1.3 ---
    // // Example: 1st Box turn at 4.8m -> LEFT
    // if (current_dist >= 10.0f && current_dist <= 5.0f)
    //     return TURN_LEFT;

    // // Example: 2nd Box turn at 5.5m -> RIGHT
    // if (current_dist >= 5.3f && current_dist <= 5.7f)
    //     return TURN_RIGHT;

    // // Example: 3rd Box turn at 6.2m -> LEFT
    // if (current_dist >= 6.0f && current_dist <= 6.4f)
    //     return TURN_LEFT;

}