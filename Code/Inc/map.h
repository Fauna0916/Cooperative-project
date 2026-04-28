#ifndef __MAP_H__
#define __MAP_H__

#include "openmv.h"
#include "odometry.h"

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
    TURN_RIGHT = -1,
    TURN_LEFT = 1,
    TURN_NONE = 0
} Turn_Direction_t;

#define DIST_MARKER_1_1 5.0f // Approx  meters from Start
#define DIST_MARKER_1_2 10.5f 
#define DIST_MARKER_1_3 13.2f
#define DIST_MARKER_1_4 7.0f
#define DIST_MARKER_1_5 8.5f

Track_Marker_t Marker_update(void);
OpenMV_Possible_Direction_t Decide_Shortest_Path(uint8_t junction_flag, uint8_t *junction_count);

#endif

