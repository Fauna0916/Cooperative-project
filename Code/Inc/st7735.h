#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"

// 你用的是 SPI4
extern SPI_HandleTypeDef hspi4; 

// 屏幕分辨率 (0.96寸 ST7735S 通常是 80x160)
#define ST7735_WIDTH   80
#define ST7735_HEIGHT  160

// 显示偏移 (很多这类小屏物理像素是从坐标 (24, 0) 或 (26, 1) 开始的)
// 如果你发现屏幕边缘有一条雪花边/花屏，就需要调整这两个值
#define ST7735_XSTART  26  
#define ST7735_YSTART  1

// RGB565 常用颜色定义
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
#define ST7735_ORANGE  0xFD20
#define ST7735_GRAY    0x8410

// 函数声明
void ST7735_Init(void);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

#endif

