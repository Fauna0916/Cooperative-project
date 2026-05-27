#ifndef __RADAR_DISPLAY_H
#define __RADAR_DISPLAY_H

#include "main.h"
#include <stdint.h>

void RadarDisplay_Init(void);

void RadarDisplay_Update(
    uint8_t mode,
    uint8_t decision,
    uint8_t locked_side,
    int8_t turn_dir,
    uint8_t left_ot2,
    uint8_t right_ot2,
    uint8_t left_state,
    uint8_t right_state,
    uint16_t left_dist,
    uint16_t right_dist,
    uint8_t left_confirmed,
    uint8_t right_confirmed
);

#endif
