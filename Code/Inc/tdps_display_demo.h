#ifndef __TDPS_DISPLAY_DEMO_H
#define __TDPS_DISPLAY_DEMO_H

#include "main.h"
#include <stdint.h>

/*
 * Real-time TFT display module for ST7735 128x160.
 *
 * Two pages with automatic switching:
 *   - LINE page (default): shows live 8-ch gray-sensor data
 *   - RADAR page: dual-radar signature, shown when is_scanning
 *     has been true for >= 500 ms; held for 2 s after scanning stops.
 */
void TDPS_DisplayDemo_Init(void);
void TDPS_DisplayDemo_Task(void);

#endif
