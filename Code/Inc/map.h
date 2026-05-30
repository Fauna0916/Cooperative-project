#ifndef __MAP_H__
#define __MAP_H__

#include "odometry.h"

#define DIST_MARKER_1_1 5.0f // Approx  meters from Start
#define DIST_MARKER_1_2 10.5f
#define DIST_MARKER_1_3 13.2f
#define DIST_MARKER_1_4 7.0f
#define DIST_MARKER_1_5 8.5f
typedef enum
{
    MARKER_START = 0,
    MARKER_1_1,
    MARKER_1_2,
    MARKER_1_3,
    MARKER_1_4,
    MARKER_1_5
} Track_Marker_t;

typedef enum
{
    Direction_RIGHT = -1,
    Direction_FORWARD,
    Direction_LEFT = 1,
    Direction_NORMAL,
} Direction_t;




Track_Marker_t Marker_update(void);
Direction_t Decide_Shortest_Path(uint8_t junction_flag);

#endif
