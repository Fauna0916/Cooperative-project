#include "radar_display.h"
#include "st7735.h"
#include <stdio.h>
#include <string.h>

#define RADAR_HIST_W   120
#define RADAR_HIST_H   14

#define HIST_L_X  30
#define HIST_L_Y  100

#define HIST_R_X  30
#define HIST_R_Y  116

#define RESULT_HOLD_MS 1000

typedef enum
{
    RADAR_UI_SCAN = 0,
    RADAR_UI_RESULT
} RadarUiPage_t;

static uint16_t hist_col = 0;
static RadarUiPage_t current_page = RADAR_UI_SCAN;
static uint32_t result_page_last_tick = 0;

static const char* mode_str(uint8_t m)
{
    switch (m)
    {
        case 0: return "LINE";
        case 1: return "SCAN";
        case 2: return "DECIDE";
        case 3: return "PASS";
        case 4: return "RETURN";
        default: return "?";
    }
}

static const char* side_str(uint8_t d)
{
    switch (d)
    {
        case 0: return "NONE";
        case 1: return "LEFT";
        case 2: return "RIGHT";
        case 3: return "BOTH";
        default: return "?";
    }
}

static const char* turn_str(int8_t dir)
{
    switch (dir)
    {
        case -1: return "RIGHT";
        case 0:  return "FWD";
        case 1:  return "LEFT";
        case 2:  return "NORMAL";
        default: return "?";
    }
}

static const char* det_str(uint8_t confirmed)
{
    return confirmed ? "DET" : "CLR";
}

static uint16_t side_color(uint8_t side)
{
    switch (side)
    {
        case 1: return ST7735_YELLOW; // LEFT
        case 2: return ST7735_CYAN;   // RIGHT
        case 3: return ST7735_RED;    // BOTH
        default: return ST7735_WHITE;
    }
}

static uint16_t turn_color(int8_t dir)
{
    switch (dir)
    {
        case -1: return ST7735_CYAN;   // RIGHT
        case 1:  return ST7735_YELLOW; // LEFT
        case 0:  return ST7735_GREEN;  // FWD
        default: return ST7735_WHITE;
    }
}

static void clear_text_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    ST7735_FillRect(x, y, w, h, ST7735_BLACK);
}

static void draw_history_frame(void)
{
    ST7735_DrawRect(HIST_L_X - 1, HIST_L_Y - 1, RADAR_HIST_W + 2, RADAR_HIST_H + 2, ST7735_WHITE);
    ST7735_DrawRect(HIST_R_X - 1, HIST_R_Y - 1, RADAR_HIST_W + 2, RADAR_HIST_H + 2, ST7735_WHITE);

    ST7735_WriteString(2, HIST_L_Y + 3, "L", ST7735_YELLOW, ST7735_BLACK, 1);
    ST7735_WriteString(2, HIST_R_Y + 3, "R", ST7735_CYAN, ST7735_BLACK, 1);
}

static void draw_history_column(uint16_t x, uint16_t y, uint16_t col, uint8_t h, uint16_t color)
{
    ST7735_DrawFastVLine(x + col, y, RADAR_HIST_H, ST7735_BLACK);

    if (h > 0)
    {
        if (h > RADAR_HIST_H) h = RADAR_HIST_H;
        ST7735_DrawFastVLine(x + col, y + (RADAR_HIST_H - h), h, color);
    }
}

static void draw_scan_page_static(void)
{
    ST7735_FillScreen(ST7735_BLACK);

    ST7735_WriteString(2,  2,  "MODE:", ST7735_GREEN,  ST7735_BLACK, 1);
    ST7735_WriteString(2,  18, "SCAN PAGE", ST7735_WHITE, ST7735_BLACK, 1);

    ST7735_WriteString(2,  38, "L:", ST7735_YELLOW, ST7735_BLACK, 1);
    ST7735_WriteString(2,  52, "R:", ST7735_CYAN,   ST7735_BLACK, 1);

    ST7735_WriteString(2,  70, "OBS :", ST7735_WHITE, ST7735_BLACK, 1);
    ST7735_WriteString(2,  84, "TURN:", ST7735_WHITE, ST7735_BLACK, 1);

    draw_history_frame();
}

static void draw_result_page_static(void)
{
    ST7735_FillScreen(ST7735_BLACK);

    ST7735_WriteString(2, 2, "MODE:", ST7735_GREEN, ST7735_BLACK, 1);

    ST7735_WriteString(2, 20, "OBS:", ST7735_WHITE, ST7735_BLACK, 1);
    ST7735_WriteString(2, 56, "TURN:", ST7735_WHITE, ST7735_BLACK, 1);

    ST7735_WriteString(2, 84, "L:", ST7735_YELLOW, ST7735_BLACK, 1);
    ST7735_WriteString(82, 84, "R:", ST7735_CYAN, ST7735_BLACK, 1);

    draw_history_frame();
}

static void switch_page_if_needed(RadarUiPage_t new_page)
{
    if (new_page == current_page)
        return;

    current_page = new_page;
    hist_col = 0;

    if (current_page == RADAR_UI_SCAN)
        draw_scan_page_static();
    else
        draw_result_page_static();
}

static void update_scan_page(
    uint8_t mode,
    uint8_t decision,
    int8_t turn_dir,
    uint16_t left_dist,
    uint16_t right_dist,
    uint8_t left_confirmed,
    uint8_t right_confirmed
)
{
    char buf[24];

    clear_text_region(40, 2, 70, 10);
    ST7735_WriteString(40, 2, mode_str(mode), ST7735_GREEN, ST7735_BLACK, 1);

    clear_text_region(18, 38, 138, 10);
    if (left_confirmed && left_dist > 0)
        snprintf(buf, sizeof(buf), "%3dcm %s", left_dist, det_str(left_confirmed));
    else
        snprintf(buf, sizeof(buf), "---  %s", det_str(left_confirmed));
    ST7735_WriteString(18, 38, buf, ST7735_YELLOW, ST7735_BLACK, 1);

    clear_text_region(18, 52, 138, 10);
    if (right_confirmed && right_dist > 0)
        snprintf(buf, sizeof(buf), "%3dcm %s", right_dist, det_str(right_confirmed));
    else
        snprintf(buf, sizeof(buf), "---  %s", det_str(right_confirmed));
    ST7735_WriteString(18, 52, buf, ST7735_CYAN, ST7735_BLACK, 1);

    clear_text_region(42, 70, 80, 10);
    ST7735_WriteString(42, 70, side_str(decision), side_color(decision), ST7735_BLACK, 1);

    clear_text_region(42, 84, 80, 10);
    ST7735_WriteString(42, 84, turn_str(turn_dir), turn_color(turn_dir), ST7735_BLACK, 1);
}

static void update_result_page(
    uint8_t mode,
    uint8_t decision,
    uint8_t locked_side,
    int8_t turn_dir,
    uint16_t left_dist,
    uint16_t right_dist,
    uint8_t left_confirmed,
    uint8_t right_confirmed
)
{
    char buf[24];
    uint8_t final_side = locked_side ? locked_side : decision;

    clear_text_region(40, 2, 70, 10);
    ST7735_WriteString(40, 2, mode_str(mode), ST7735_GREEN, ST7735_BLACK, 1);

    clear_text_region(2, 30, 156, 22);
    ST7735_WriteString(28, 30, side_str(final_side), side_color(final_side), ST7735_BLACK, 2);

    clear_text_region(2, 66, 156, 22);
    ST7735_WriteString(24, 66, turn_str(turn_dir), turn_color(turn_dir), ST7735_BLACK, 2);

    clear_text_region(16, 84, 60, 10);
    if (left_confirmed && left_dist > 0)
        snprintf(buf, sizeof(buf), "%2dcm %s", left_dist, det_str(left_confirmed));
    else
        snprintf(buf, sizeof(buf), "--  %s", det_str(left_confirmed));
    ST7735_WriteString(16, 84, buf, ST7735_YELLOW, ST7735_BLACK, 1);

    clear_text_region(96, 84, 60, 10);
    if (right_confirmed && right_dist > 0)
        snprintf(buf, sizeof(buf), "%2dcm %s", right_dist, det_str(right_confirmed));
    else
        snprintf(buf, sizeof(buf), "--  %s", det_str(right_confirmed));
    ST7735_WriteString(96, 84, buf, ST7735_CYAN, ST7735_BLACK, 1);
}

static void update_history(uint16_t left_dist, uint16_t right_dist, uint8_t left_confirmed, uint8_t right_confirmed)
{
    uint8_t hL = 0;
    uint8_t hR = 0;

    if (left_confirmed && left_dist > 0 && left_dist <= 60)
    {
        hL = RADAR_HIST_H - (left_dist * RADAR_HIST_H / 60);
        if (hL < 2) hL = 2;
    }

    if (right_confirmed && right_dist > 0 && right_dist <= 60)
    {
        hR = RADAR_HIST_H - (right_dist * RADAR_HIST_H / 60);
        if (hR < 2) hR = 2;
    }

    draw_history_column(HIST_L_X, HIST_L_Y, hist_col, hL, ST7735_YELLOW);
    draw_history_column(HIST_R_X, HIST_R_Y, hist_col, hR, ST7735_CYAN);

    hist_col++;
    if (hist_col >= RADAR_HIST_W)
        hist_col = 0;
}

void RadarDisplay_Init(void)
{
    hist_col = 0;
    current_page = RADAR_UI_SCAN;
    result_page_last_tick = 0;
    draw_scan_page_static();
}

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
)
{
    (void)left_ot2;
    (void)right_ot2;
    (void)left_state;
    (void)right_state;

    uint32_t now = HAL_GetTick();

    uint8_t result_trigger =
        (locked_side != 0) ||
        (decision != 0) ||
        (mode == 2) ||   // MODE_OBSTACLE_DECIDE
        (mode == 3) ||   // MODE_OBSTACLE_PASS
        (mode == 4);     // MODE_RETURN_LINE

    RadarUiPage_t new_page = current_page;

    if (result_trigger)
    {
        new_page = RADAR_UI_RESULT;
        result_page_last_tick = now;
    }
    else
    {
        if (current_page == RADAR_UI_RESULT)
        {
            if ((now - result_page_last_tick) < RESULT_HOLD_MS)
                new_page = RADAR_UI_RESULT;
            else
                new_page = RADAR_UI_SCAN;
        }
        else
        {
            new_page = RADAR_UI_SCAN;
        }
    }

    switch_page_if_needed(new_page);

    if (current_page == RADAR_UI_SCAN)
    {
        update_scan_page(
            mode,
            decision,
            turn_dir,
            left_dist,
            right_dist,
            left_confirmed,
            right_confirmed
        );
    }
    else
    {
        update_result_page(
            mode,
            decision,
            locked_side,
            turn_dir,
            left_dist,
            right_dist,
            left_confirmed,
            right_confirmed
        );
    }

    update_history(left_dist, right_dist, left_confirmed, right_confirmed);
}
