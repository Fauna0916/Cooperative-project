#include "display_ui.h"

// ==================== 内部状态变量 ====================
static DisplayMode_t g_displayMode = DISPLAY_MODE_LINE;

static uint8_t g_l1 = 0;
static uint8_t g_l2 = 0;
static uint8_t g_r1 = 0;
static uint8_t g_r2 = 0;

static uint8_t g_radarLeft = 0;
static uint8_t g_radarRight = 0;

// ==================== 内部绘图函数 ====================

// 画单个传感器状态块
static void UI_DrawSensorBlock(uint16_t x, uint16_t y, uint16_t color)
{
    ST7735_FillRect(x, y, 14, 14, ST7735_WHITE);
    ST7735_FillRect(x + 2, y + 2, 10, 10, color);
}

// 画柱状图
static void UI_DrawBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t value, uint16_t color)
{
    ST7735_FillRect(x, y, w, h, ST7735_WHITE);
    ST7735_FillRect(x + 2, y + 2, w - 4, h - 4, ST7735_BLACK);

    if (value > 100) value = 100;

    uint16_t fill_h = (uint16_t)((h - 4) * value / 100);

    if (fill_h > 0)
    {
        ST7735_FillRect(x + 2, y + h - 2 - fill_h, w - 4, fill_h, color);
    }
}

// ==================== 页面绘制函数 ====================

// 巡线页面
static void UI_DrawLinePage(void)
{
    uint16_t color_l1 = g_l1 ? ST7735_GREEN : ST7735_GRAY;
    uint16_t color_l2 = g_l2 ? ST7735_GREEN : ST7735_GRAY;
    uint16_t color_r1 = g_r1 ? ST7735_GREEN : ST7735_GRAY;
    uint16_t color_r2 = g_r2 ? ST7735_GREEN : ST7735_GRAY;

    ST7735_FillScreen(ST7735_BLACK);

    // 顶部四个状态块
    UI_DrawSensorBlock(4,  20, color_l1);
    UI_DrawSensorBlock(22, 20, color_l2);
    UI_DrawSensorBlock(40, 20, color_r1);
    UI_DrawSensorBlock(58, 20, color_r2);

    // 车体示意
    ST7735_FillRect(20, 70, 40, 20, ST7735_BLUE);

    // 底部传感器条
    ST7735_FillRect(8,  55, 10, 6, color_l1);
    ST7735_FillRect(26, 55, 10, 6, color_l2);
    ST7735_FillRect(44, 55, 10, 6, color_r1);
    ST7735_FillRect(62, 55, 10, 6, color_r2);

    // 页面底部标识条
    ST7735_FillRect(10, 125, 60, 8, ST7735_BLUE);
}

// 雷达页面
static void UI_DrawRadarPage(void)
{
    ST7735_FillScreen(ST7735_BLACK);

    // 中间参考线
    ST7735_FillRect(39, 10, 2, 100, ST7735_GRAY);

    // 左右柱状条
    UI_DrawBar(10, 20, 20, 80, g_radarLeft, ST7735_GREEN);
    UI_DrawBar(50, 20, 20, 80, g_radarRight, ST7735_RED);

    // 底部状态条
    ST7735_FillRect(10, 125, 60, 8, ST7735_ORANGE);
}

// ==================== 对外接口 ====================

void Display_Init(void)
{
    ST7735_Init();
    g_displayMode = DISPLAY_MODE_LINE;

    g_l1 = 0;
    g_l2 = 1;
    g_r1 = 1;
    g_r2 = 0;

    g_radarLeft = 20;
    g_radarRight = 80;
}

void Display_SetMode(DisplayMode_t mode)
{
    g_displayMode = mode;
}

void Display_SetLineData(uint8_t l1, uint8_t l2, uint8_t r1, uint8_t r2)
{
    g_l1 = l1;
    g_l2 = l2;
    g_r1 = r1;
    g_r2 = r2;
}

void Display_SetRadarData(uint8_t left_strength, uint8_t right_strength)
{
    g_radarLeft = left_strength;
    g_radarRight = right_strength;
}

void Display_Update(void)
{
    if (g_displayMode == DISPLAY_MODE_LINE)
    {
        UI_DrawLinePage();
    }
    else if (g_displayMode == DISPLAY_MODE_RADAR)
    {
        UI_DrawRadarPage();
    }
}

void Display_Demo(void)
{
    // 巡线页面几组假数据
    Display_SetMode(DISPLAY_MODE_LINE);

    Display_SetLineData(0, 1, 1, 0);
    Display_Update();
    HAL_Delay(800);

    Display_SetLineData(0, 1, 0, 0);
    Display_Update();
    HAL_Delay(800);

    Display_SetLineData(0, 0, 1, 0);
    Display_Update();
    HAL_Delay(800);

    Display_SetLineData(1, 1, 0, 0);
    Display_Update();
    HAL_Delay(800);

    // 切换到雷达页面
    Display_SetMode(DISPLAY_MODE_RADAR);

    Display_SetRadarData(20, 80);
    Display_Update();
    HAL_Delay(800);

    Display_SetRadarData(40, 60);
    Display_Update();
    HAL_Delay(800);

    Display_SetRadarData(70, 30);
    Display_Update();
    HAL_Delay(800);

    Display_SetRadarData(90, 10);
    Display_Update();
    HAL_Delay(800);
}
