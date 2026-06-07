#ifndef GRAY_SENSOR_16CH_H
#define GRAY_SENSOR_16CH_H

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"
#include "map.h"

/*
 * 16-channel gray sensor driver (future upgrade path).
 *
 * Hardware assumption:
 *   4 address lines (AD0..AD3) + 1 comparator output (GRAY_OUT).
 *   AD0-AD2:  channel select within an 8-ch bank (0..7)
 *   AD3:      bank select (0 = lower 8, 1 = upper 8)
 *   GRAY_OUT: comparator output (1 = black line detected)
 *
 * Interface mirrors the 8-ch version (gray_sensor.h) so the
 * control layer can swap with minimal changes.
 */

#define LEFT_ERR_16  (-200)
#define RIGHT_ERR_16  (200)

typedef enum
{
    GraySensor16_FLAG_NORMAL = 0x00,
    GraySensor16_FLAG_JUNC   = 0x10,
    GraySensor16_FLAG_LOST   = 0xFF
} GraySensor16_Flag_t;

typedef struct
{
    uint8_t  flag;       /* junction / lost / normal          */
    int16_t  err_f;      /* filtered centre error (-200..200)  */
    int16_t  err_l;      /* left-branch error                  */
    int16_t  err_r;      /* right-branch error                 */
    uint16_t raw_data;   /* all 16 bits, bit15 = leftmost     */
} GraySensor16_Data_t;

void     GraySensor16_Init(void);
void     GraySensor16_Update(void);
void     GraySensor16_ForceSetLastErr(int16_t forced_err);
GraySensor16_Data_t *GraySensor16_GetData(void);

#endif /* GRAY_SENSOR_16CH_H */
