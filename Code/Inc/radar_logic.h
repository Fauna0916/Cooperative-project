#ifndef RADAR_LOGIC_H
#define RADAR_LOGIC_H

#include "radar_driver.h"
#include "map.h"

typedef enum
{
    RADAR_NONE = 0,
    RADAR_LEFT,
    RADAR_RIGHT,
    RADAR_BOTH
} RadarDecision;

typedef struct
{
    uint8_t left_candidate;
    uint8_t left_stable_count;
    uint8_t left_confirmed;

    uint8_t right_candidate;
    uint8_t right_stable_count;
    uint8_t right_confirmed;
} RadarLogic_t;

void RadarLogic_Init(void);
RadarDecision RadarLogic_Update(RadarLogic_t *logic, RadarDriver_t *left, RadarDriver_t *right);
Direction_t Radar_GetTurnDirection(RadarDecision decision);

#endif
