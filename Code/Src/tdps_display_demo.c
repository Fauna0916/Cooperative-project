#include "tdps_display_demo.h"
#include "gray_sensor.h"
#include "radar.h"
#include "odometry.h"
#include "robot_task.h"
#include "st7735.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * TDPS Display — Real sensor data on ST7735 128x160
 * ============================================================
 * Auto-switch logic:
 *   - LINE page by default
 *   - RADAR page when is_scanning has been true for >= 500 ms
 *   - Hold RADAR page for 2 s after scanning stops
 * ============================================================ */

#define DISP_REFRESH_MS       100U
#define RADAR_ACTIVATE_MS     500U   /* is_scanning must be true this long */
#define RADAR_HOLD_MS        2000U   /* keep radar page after scan ends   */

typedef enum
{
    DISP_PAGE_LINE = 0,
    DISP_PAGE_RADAR
} DisplayPage_t;

static DisplayPage_t current_page = DISP_PAGE_LINE;
static DisplayPage_t last_drawn_page = (DisplayPage_t)255;
static uint32_t last_refresh_tick = 0;
static uint32_t scan_true_since = 0;   /* tick when is_scanning first went true */
static uint32_t scan_false_since = 0;  /* tick when is_scanning first went false */
static bool was_scanning = false;

/* -------- helpers ------------------------------------------------------ */

static void disp_force_redraw(void)
{
    last_drawn_page = (DisplayPage_t)255;
}

/* -------- auto-switch state machine ------------------------------------ */

static void disp_update_page_state(void)
{
    /*
     * Only show the RADAR page when all three conditions hold:
     *   1. Radar hardware is actually scanning
     *   2. Pre-scan has finished (don't react to the 5-s sampling window)
     *   3. We are inside the task-3 zone (MARKER_1_4, not yet done)
     */
    bool scanning = Radar_IsScanning()
                    && !RobotTask_IsPreScanActive()
                    && RobotTask_IsInTask3Zone();
    uint32_t now = HAL_GetTick();

    /* Rising edge: scanning just started */
    if (scanning && !was_scanning)
    {
        scan_true_since = now;
        scan_false_since = 0;
    }
    /* Falling edge: scanning just stopped */
    else if (!scanning && was_scanning)
    {
        scan_false_since = now;
        scan_true_since = 0;
    }

    /* State transitions */
    if (current_page == DISP_PAGE_LINE)
    {
        if (scanning && scan_true_since != 0 &&
            (now - scan_true_since) >= RADAR_ACTIVATE_MS)
        {
            current_page = DISP_PAGE_RADAR;
            disp_force_redraw();
        }
    }
    else /* DISP_PAGE_RADAR */
    {
        if (!scanning && scan_false_since != 0 &&
            (now - scan_false_since) >= RADAR_HOLD_MS)
        {
            current_page = DISP_PAGE_LINE;
            disp_force_redraw();
        }
    }

    was_scanning = scanning;
}

/* -------- LINE page ---------------------------------------------------- */

static void disp_draw_line_static(void)
{
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_FillRect(0, 0, ST7735_TFTWIDTH, 12, ST7735_BLUE);
    ST7735_WriteString(2, 2, "LINE FOLLOW", ST7735_WHITE, ST7735_BLUE, 1);

    /* Sensor channel labels */
    ST7735_WriteString(2, 16, "Ch:0 1 2 3 4 5 6 7", ST7735_GRAY, ST7735_BLACK, 1);

    /* Labels for data area */
    ST7735_WriteString(2, 58, "Raw:", ST7735_GRAY, ST7735_BLACK, 1);
    ST7735_WriteString(2, 70, "Err:", ST7735_GRAY, ST7735_BLACK, 1);
    ST7735_WriteString(2, 82, "Flag:", ST7735_GRAY, ST7735_BLACK, 1);

    /* Divider line */
    ST7735_DrawFastHLine(0, 100, ST7735_TFTWIDTH, ST7735_GRAY);

    /* Status footer */
    ST7735_WriteString(2, 104, "Marker:", ST7735_GRAY, ST7735_BLACK, 1);
    ST7735_WriteString(2, 118, "Dist:", ST7735_GRAY, ST7735_BLACK, 1);
    ST7735_WriteString(2, 132, "Spd:", ST7735_GRAY, ST7735_BLACK, 1);

    /* Key hint */
    ST7735_WriteString(2, 150, "K1:T3", ST7735_GRAY, ST7735_BLACK, 1);
}

static void disp_draw_line_dynamic(const GraySensor_Data_t *gray,
                                    float distance, float speed)
{
    char buf[32];

    /* ---- Sensor bar (8 blocks, 14px wide each, at y=28) ---- */
    for (uint8_t i = 0; i < 8; i++)
    {
        uint16_t x = 4 + i * 15;
        uint8_t bit = (gray->raw_data >> (7 - i)) & 0x01;
        uint16_t color = bit ? ST7735_WHITE : ST7735_BLACK;
        ST7735_FillRect(x, 28, 13, 16, color);
        if (!bit)
            ST7735_DrawRect(x, 28, 13, 16, ST7735_GRAY);
    }

    /* ---- Raw hex ---- */
    sprintf(buf, "Raw: 0x%02X", gray->raw_data);
    ST7735_WriteString(30, 58, buf, ST7735_WHITE, ST7735_BLACK, 1);

    /* ---- Error ---- */
    sprintf(buf, "Err: %d  ", gray->err_f);
    ST7735_WriteString(30, 70, buf,
                       (gray->flag == GraySensor_FLAG_LOST) ? ST7735_RED : ST7735_GREEN,
                       ST7735_BLACK, 1);

    /* ---- Flag ---- */
    const char *flag_str;
    uint16_t flag_color;
    if (gray->flag == GraySensor_FLAG_LOST)
    {
        flag_str = "LOST";
        flag_color = ST7735_RED;
    }
    else if ((gray->flag & 0xF0) == GraySensor_FLAG_JUNC)
    {
        sprintf(buf, "JUNC 0x%02X", gray->flag);
        flag_str = buf;
        flag_color = ST7735_YELLOW;
    }
    else
    {
        flag_str = "NORMAL";
        flag_color = ST7735_GREEN;
    }
    ST7735_WriteString(30, 82, flag_str, flag_color, ST7735_BLACK, 1);

    // /* ---- Marker (from odometry distance) ---- */
    // sprintf(buf, "Mkr: --");
    // ST7735_WriteString(48, 104, buf, ST7735_WHITE, ST7735_BLACK, 1);

    // /* ---- Distance ---- */
    // sprintf(buf, "Dist: %.2f m", (double)distance);
    // ST7735_WriteString(48, 118, buf, ST7735_WHITE, ST7735_BLACK, 1);

    // /* ---- Speed ---- */
    // sprintf(buf, "Spd: %.2f m/s", (double)speed);
    // ST7735_WriteString(48, 132, buf, ST7735_WHITE, ST7735_BLACK, 1);
}

static void disp_update_line_page(void)
{
    GraySensor_Data_t *gray = GraySensor_GetData();
    Odometry_State_t *odo = Odometry_GetState();
    float distance = odo->distance;
    float speed = odo->linear_vel;

    disp_draw_line_dynamic(gray, distance, speed);
}

/* -------- RADAR page --------------------------------------------------- */

static void disp_draw_radar_static(void)
{
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_FillRect(0, 0, ST7735_TFTWIDTH, 12, ST7735_RED);
    ST7735_WriteString(8, 2, "RADAR SCAN", ST7735_WHITE, ST7735_RED, 1);

    /* Left radar section */
    ST7735_WriteString(2, 18, "L-Radar", ST7735_CYAN, ST7735_BLACK, 1);
    ST7735_DrawRect(2, 32, 56, 40, ST7735_GRAY);

    /* Right radar section */
    ST7735_WriteString(70, 18, "R-Radar", ST7735_CYAN, ST7735_BLACK, 1);
    ST7735_DrawRect(70, 32, 56, 40, ST7735_GRAY);

    /* Divider */
    ST7735_DrawFastHLine(0, 80, ST7735_TFTWIDTH, ST7735_GRAY);

    /* Decision area */
    ST7735_WriteString(2, 86, "Decision:", ST7735_GRAY, ST7735_BLACK, 1);

    /* Vote area */
    ST7735_WriteString(2, 106, "Votes L:", ST7735_GRAY, ST7735_BLACK, 1);
    ST7735_WriteString(2, 118, "Votes R:", ST7735_GRAY, ST7735_BLACK, 1);

    /* Mode hint */
    ST7735_WriteString(2, 150, "scanning...", ST7735_YELLOW, ST7735_BLACK, 1);
}

static void disp_draw_radar_dynamic(void)
{
    char buf[32];
    uint16_t left_dist = Radar_GetLeftDistance();
    uint16_t right_dist = Radar_GetRightDistance();
    bool left_tgt = Radar_GetLeftHasTarget();
    bool right_tgt = Radar_GetRightHasTarget();
    uint16_t left_votes = Radar_GetLeftVotes();
    uint16_t right_votes = Radar_GetRightVotes();
    Direction_t decision = Radar_GetAvoidanceDirection();

    /* ---- Left radar box ---- */
    uint16_t l_color = left_tgt ? ST7735_RED : ST7735_GREEN;
    ST7735_FillRect(4, 34, 52, 10, ST7735_BLACK);
    sprintf(buf, "%s", left_tgt ? "TGT" : "CLR");
    ST7735_WriteString(4, 35, buf, l_color, ST7735_BLACK, 1);

    ST7735_FillRect(4, 46, 52, 10, ST7735_BLACK);
    sprintf(buf, "%u cm", left_dist);
    ST7735_WriteString(4, 47, buf, ST7735_WHITE, ST7735_BLACK, 1);

    /* Distance bar */
    uint16_t bar_w = (left_dist > 80) ? 52 : (uint16_t)((uint32_t)left_dist * 52 / 80);
    ST7735_FillRect(4, 58, bar_w, 6, l_color);
    ST7735_FillRect(4 + bar_w, 58, 52 - bar_w, 6, ST7735_BLACK);
    ST7735_DrawRect(4, 58, 52, 6, ST7735_GRAY);

    /* ---- Right radar box ---- */
    uint16_t r_color = right_tgt ? ST7735_RED : ST7735_GREEN;
    ST7735_FillRect(72, 34, 52, 10, ST7735_BLACK);
    sprintf(buf, "%s", right_tgt ? "TGT" : "CLR");
    ST7735_WriteString(72, 35, buf, r_color, ST7735_BLACK, 1);

    ST7735_FillRect(72, 46, 52, 10, ST7735_BLACK);
    sprintf(buf, "%u cm", right_dist);
    ST7735_WriteString(72, 47, buf, ST7735_WHITE, ST7735_BLACK, 1);

    /* Distance bar */
    bar_w = (right_dist > 80) ? 52 : (uint16_t)((uint32_t)right_dist * 52 / 80);
    ST7735_FillRect(72, 58, bar_w, 6, r_color);
    ST7735_FillRect(72 + bar_w, 58, 52 - bar_w, 6, ST7735_BLACK);
    ST7735_DrawRect(72, 58, 52, 6, ST7735_GRAY);

    /* ---- Decision ---- */
    const char *dec_str;
    uint16_t dec_color;
    switch (decision)
    {
    case Direction_LEFT:
        dec_str = "TURN LEFT  ";
        dec_color = ST7735_CYAN;
        break;
    case Direction_RIGHT:
        dec_str = "TURN RIGHT ";
        dec_color = ST7735_CYAN;
        break;
    case Direction_NORMAL:
        dec_str = "...waiting";
        dec_color = ST7735_YELLOW;
        break;
    default:
        dec_str = "FORWARD    ";
        dec_color = ST7735_GREEN;
        break;
    }

    ST7735_FillRect(50, 86, 76, 10, ST7735_BLACK);
    ST7735_WriteString(50, 86, dec_str, dec_color, ST7735_BLACK, 1);

    /* ---- Votes ---- */
    ST7735_FillRect(50, 106, 76, 10, ST7735_BLACK);
    sprintf(buf, "%u / %u", left_votes, (unsigned int)30);
    ST7735_WriteString(50, 106, buf, ST7735_WHITE, ST7735_BLACK, 1);

    ST7735_FillRect(50, 118, 76, 10, ST7735_BLACK);
    sprintf(buf, "%u / %u", right_votes, (unsigned int)30);
    ST7735_WriteString(50, 118, buf, ST7735_WHITE, ST7735_BLACK, 1);

    /* ---- Mode hint at bottom ---- */
    ST7735_FillRect(2, 150, 120, 8, ST7735_BLACK);
    if (Radar_IsScanning())
    {
        ST7735_WriteString(2, 150, "scanning...", ST7735_YELLOW, ST7735_BLACK, 1);
    }
    else
    {
        ST7735_WriteString(2, 150, "frozen", ST7735_GRAY, ST7735_BLACK, 1);
    }
}

static void disp_update_radar_page(void)
{
    disp_draw_radar_dynamic();
}

/* -------- public API --------------------------------------------------- */

void TDPS_DisplayDemo_Init(void)
{
    current_page = DISP_PAGE_LINE;
    last_drawn_page = (DisplayPage_t)255;
    last_refresh_tick = 0;
    scan_true_since = 0;
    scan_false_since = 0;
    was_scanning = false;

    /* Force first draw */
    disp_force_redraw();
}

void TDPS_DisplayDemo_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Run auto-switch state machine every call (cheap) */
    disp_update_page_state();

    /* 2. Redraw static chrome on page change */
    if (last_drawn_page != current_page)
    {
        if (current_page == DISP_PAGE_LINE)
            disp_draw_line_static();
        else
            disp_draw_radar_static();

        last_drawn_page = current_page;
    }

    /* 3. Rate-limited dynamic update */
    if ((now - last_refresh_tick) < DISP_REFRESH_MS)
        return;

    last_refresh_tick = now;

    if (current_page == DISP_PAGE_LINE)
    {
        disp_update_line_page();
    }
    else
    {
        disp_update_radar_page();
    }
}
