#include "gray_sensor_16ch.h"
#include <stdlib.h>
#include <math.h>

/* ========== Configuration ========== */
#define MAX_ERR_STEP     35      /* max error change per frame (scaled for 16ch) */
#define LOST_THRESHOLD  2000     /* consecutive lost frames before declaring LOST */
#define CENTER_FLOAT     7.5f    /* floating centre: (15 + 0) / 2 = 7.5 */

/* ========== Static state ========== */
static uint32_t       lost_frame_cnt = 0;
static int16_t        last_valid_err_f = 0;
static uint8_t        force_update_flag = 0;
static GraySensor16_Data_t gray16_data = {0};

/* -------- Blob descriptor (same as 8-ch) ------------------------------- */

typedef struct
{
    int16_t center_err;
    uint8_t width;
    uint8_t is_left;
    uint8_t is_right;
} LineBlob16_t;

/* -------- Multiplexer helper ------------------------------------------- */

static inline void Multiplexer16_Delay(void)
{
    for (volatile int i = 0; i < 15; i++)
    {
        __NOP();
    }
}

/*
 * Set 4-bit address (AD0..AD3) on GPIOs.
 * AD0 = PE7, AD1 = PC5, AD2 = PC9, AD3 = user-defined.
 *
 * NOTE: AD3 must be defined in main.h (e.g. #define GRAY_AD3_Pin / _GPIO_Port).
 * If not yet wired, change the #if block below.
 */
static inline void GraySensor16_SetChannel(uint8_t ch)
{
    HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin,
                      (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin,
                      (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin,
                      (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#ifdef GRAY_AD3_Pin
    HAL_GPIO_WritePin(GRAY_AD3_GPIO_Port, GRAY_AD3_Pin,
                      (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)(ch & 0x08); /* AD3 not yet wired — only lower 8 channels active */
#endif
}

/* -------- Public API --------------------------------------------------- */

void GraySensor16_Init(void)
{
    last_valid_err_f = 0;
    force_update_flag = 0;
    memset(&gray16_data, 0, sizeof(gray16_data));
}

GraySensor16_Data_t *GraySensor16_GetData(void)
{
    return &gray16_data;
}

void GraySensor16_ForceSetLastErr(int16_t forced_err)
{
    last_valid_err_f = forced_err;
    force_update_flag = 1;
}

/* -------- Raw read (16 bits) ------------------------------------------- */

static uint16_t GraySensor16_ReadRaw(void)
{
    uint16_t raw = 0;
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        GraySensor16_SetChannel(ch);
        Multiplexer16_Delay();
        if (HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin))
        {
            raw |= (1 << (15 - ch)); /* bit15 = leftmost sensor */
        }
    }
    return raw;
}

/* -------- Direction counting (for junction detection) ------------------ */

static inline uint8_t count_directions_16(uint8_t dir_avail)
{
    uint8_t cnt = 0;
    if (dir_avail & 0x01) cnt++;
    if (dir_avail & 0x02) cnt++;
    if (dir_avail & 0x04) cnt++;
    return cnt;
}

/* -------- Main update -------------------------------------------------- */

void GraySensor16_Update(void)
{
    uint16_t current_raw = GraySensor16_ReadRaw();
    gray16_data.raw_data = current_raw;

    /* 1. Lost-line handling */
    if (current_raw == 0x0000)
    {
        if (++lost_frame_cnt < LOST_THRESHOLD)
        {
            gray16_data.flag  = GraySensor16_FLAG_NORMAL;
            gray16_data.err_f = last_valid_err_f;
        }
        else
        {
            gray16_data.flag  = GraySensor16_FLAG_LOST;
            gray16_data.err_f = (last_valid_err_f < 0) ? LEFT_ERR_16 : RIGHT_ERR_16;
        }
        gray16_data.err_l = LEFT_ERR_16;
        gray16_data.err_r = RIGHT_ERR_16;
        return;
    }
    lost_frame_cnt = 0;

    /* 2. Connected-component (blob) analysis */
    LineBlob16_t blobs[8];
    uint8_t blob_count = 0;
    int8_t current_blob_start = -1;

    /* Iterate bits 15 down to 0, with a sentinel at -1 */
    for (int8_t i = 15; i >= -1; i--)
    {
        uint8_t bit_is_1 = (i >= 0) ? ((current_raw >> i) & 0x01) : 0;
        if (bit_is_1)
        {
            if (current_blob_start == -1)
                current_blob_start = i;
        }
        else if (current_blob_start != -1)
        {
            uint8_t end = i + 1;
            float   center_bit     = (current_blob_start + end) / 2.0f;
            float   dist_from_ctr  = center_bit - CENTER_FLOAT;
            float   abs_dist       = fabsf(dist_from_ctr);

            /* Non-linear mapping (same curve, scaled x2 vs 8-ch) */
            float scaled_err = (abs_dist <= 2.5f)
                ? (abs_dist * 26.6f)
                : (66.0f + (abs_dist - 2.5f) * 30.0f);
            int16_t err = (int16_t)scaled_err;
            if (dist_from_ctr > 0) err = -err; /* left = negative */

            blobs[blob_count].center_err = err;
            blobs[blob_count].width      = current_blob_start - end + 1;
            blobs[blob_count].is_left    = (current_blob_start >= 10);
            blobs[blob_count].is_right   = (end <= 5);
            blob_count++;

            if (blob_count >= 8) break; /* safety */
            current_blob_start = -1;
        }
    }

    /* 3. Target selection (closest to last known error) */
    int16_t target_err = blobs[0].center_err;
    uint8_t best_idx   = 0;
    if (blob_count > 1)
    {
        int16_t min_diff = 999;
        for (uint8_t i = 0; i < blob_count; i++)
        {
            int16_t diff = abs(blobs[i].center_err - last_valid_err_f);
            if (diff < min_diff)
            {
                min_diff = diff;
                best_idx = i;
            }
        }
        target_err = blobs[best_idx].center_err;
    }

    /* 4. Rate-of-change limiter */
    if (force_update_flag)
    {
        last_valid_err_f = target_err;
        force_update_flag = 0;
    }
    else
    {
        int16_t delta = target_err - last_valid_err_f;
        if (delta > MAX_ERR_STEP)       delta = MAX_ERR_STEP;
        if (delta < -MAX_ERR_STEP)      delta = -MAX_ERR_STEP;
        last_valid_err_f += delta;
    }

    /* 5. Output */
    gray16_data.err_f = last_valid_err_f;

    gray16_data.err_l = (blobs[best_idx].center_err < -40)
        ? blobs[best_idx].center_err : LEFT_ERR_16;
    gray16_data.err_r = (blobs[best_idx].center_err > 40)
        ? blobs[best_idx].center_err : RIGHT_ERR_16;

    /* 6. Junction detection (same heuristic, wider thresholds) */
    uint8_t dir_avail = 0;
    if (blobs[best_idx].width >= 4)
    {
        if (blobs[best_idx].is_left)                           dir_avail |= 0x01;
        if (abs(blobs[best_idx].center_err) < 90)              dir_avail |= 0x02;
        if (blobs[best_idx].is_right)                          dir_avail |= 0x04;
        if (blobs[best_idx].width >= 8)                        dir_avail  = 0x07;
    }

    if (count_directions_16(dir_avail) >= 2)
    {
        gray16_data.flag = GraySensor16_FLAG_JUNC | (dir_avail & 0x0F);
    }
    else
    {
        gray16_data.flag = GraySensor16_FLAG_NORMAL;
    }
}
