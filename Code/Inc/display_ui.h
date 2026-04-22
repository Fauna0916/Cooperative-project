#ifndef __DISPLAY_UI_H
#define __DISPLAY_UI_H

#include "st7735.h"
#include "main.h"

// 显示模式
typedef enum
{
    DISPLAY_MODE_LINE = 0,
    DISPLAY_MODE_RADAR
} DisplayMode_t;

// 初始化显示管理
void Display_Init(void);

// 设置显示模式
void Display_SetMode(DisplayMode_t mode);

// 更新显示（根据当前模式刷新页面）
void Display_Update(void);

// 设置巡线数据
void Display_SetLineData(uint8_t l1, uint8_t l2, uint8_t r1, uint8_t r2);

// 设置雷达数据
void Display_SetRadarData(uint8_t left_strength, uint8_t right_strength);

// 演示函数：自动切换页面
void Display_Demo(void);

#endif
