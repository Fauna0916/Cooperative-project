#ifndef __LINE_DISPLAY_H
#define __LINE_DISPLAY_H

#include "main.h"
#include <stdint.h>

void LineDisplay_Init(void);

void LineDisplay_Update(
    const uint16_t sensor_value[8],
    const uint8_t sensor_detected[8],
    int16_t line_pos,
    int16_t line_err
);

#endif
