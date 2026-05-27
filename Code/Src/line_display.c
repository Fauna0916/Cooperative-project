#include "line_display.h"
#include "st7735.h"
#include <stdio.h>
#include <string.h>

#define BAR_W   12
#define BAR_GAP 6
#define BAR_X0  10
#define BAR_Y0  28
#define BAR_H   50

#define BIN_Y   84
#define INFO_Y  104

static void clear_text_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    ST7735_FillRect(x, y, w, h, ST7735_BLACK);
}

void LineDisplay_Init(void)
{
    ST7735_FillScreen(ST7735_BLACK);

    ST7735_WriteString(2, 2, "MODE:", ST7735_GREEN, ST7735_BLACK, 1);
    ST7735_WriteString(42, 2, "LINE", ST7735_GREEN, ST7735_BLACK, 1);

    // 柱状图边框
    ST7735_DrawRect(4, 22, 152, 58, ST7735_WHITE);

    // 二值区域标签
    ST7735_WriteString(2, BIN_Y, "BIN:", ST7735_WHITE, ST7735_BLACK, 1);

    // 信息标签
    ST7735_WriteString(2, INFO_Y, "POS:", ST7735_YELLOW, ST7735_BLACK, 1);
    ST7735_WriteString(80, INFO_Y, "ERR:", ST7735_CYAN, ST7735_BLACK, 1);
}

void LineDisplay_Update(
    const uint16_t sensor_value[8],
    const uint8_t sensor_detected[8],
    int16_t line_pos,
    int16_t line_err
)
{
    char buf[24];

    // ===== 1. 刷 8 路柱状图 =====
    for (int i = 0; i < 8; i++)
    {
        uint16_t x = BAR_X0 + i * (BAR_W + BAR_GAP);

        // 清该柱区域
        ST7735_FillRect(x, BAR_Y0, BAR_W, BAR_H, ST7735_BLACK);

        // 假设传感器值范围 0~1000，映射到 BAR_H
        uint16_t h = sensor_value[i];
        if (h > 1000) h = 1000;
        h = (h * BAR_H) / 1000;

        if (h > 0)
        {
            ST7735_FillRect(x, BAR_Y0 + (BAR_H - h), BAR_W, h, ST7735_YELLOW);
        }
    }

    // ===== 2. 刷二值结果 =====
    for (int i = 0; i < 8; i++)
    {
        uint16_t x = 34 + i * 14;
        uint16_t color = sensor_detected[i] ? ST7735_GREEN : ST7735_RED;
        ST7735_FillRect(x, BIN_Y, 10, 10, color);
    }

    // ===== 3. 刷位置和误差 =====
    clear_text_region(34, INFO_Y, 40, 10);
    snprintf(buf, sizeof(buf), "%d", line_pos);
    ST7735_WriteString(34, INFO_Y, buf, ST7735_YELLOW, ST7735_BLACK, 1);

    clear_text_region(112, INFO_Y, 40, 10);
    snprintf(buf, sizeof(buf), "%d", line_err);
    ST7735_WriteString(112, INFO_Y, buf, ST7735_CYAN, ST7735_BLACK, 1);
}
