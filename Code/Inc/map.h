#ifndef __MAP_H__
#define __MAP_H__

#include "odometry.h"

#define DIST_MARKER_1_1 5.0f // Approx  meters from Start
#define DIST_MARKER_1_2 10.5f
#define DIST_MARKER_1_3 13.2f
#define DIST_MARKER_1_4 15.0f
#define DIST_MARKER_1_5 20.0f
typedef enum
{
    MARKER_START = 0,
    MARKER_1_1,
    MARKER_1_2,
    MARKER_1_3,
    MARKER_1_4,
    MARKER_1_5
} Track_Marker_t;

typedef struct
{
    float x;
    float y;
    float dist;
} Marker_Info_t;

static const Marker_Info_t MAP_MARKERS[] = {
    [MARKER_START] = {0.0f, 0.0f, 0.0f},
    [MARKER_1_1] = {-1.5f, 2.0f, DIST_MARKER_1_1},
    [MARKER_1_2] = {-2.0f, 5.0f, DIST_MARKER_1_2},
    [MARKER_1_3] = {0.0f, 8.0f, DIST_MARKER_1_3},
    [MARKER_1_4] = {4.9f, 4.1f, DIST_MARKER_1_4},
    [MARKER_1_5] = {0.0f, 10.0f, DIST_MARKER_1_5}};

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
