#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"
#include <stdint.h>

#define ST7735_TFTWIDTH   128
#define ST7735_TFTHEIGHT  160

extern uint16_t ST7735_WIDTH;
extern uint16_t ST7735_HEIGHT;

/* RGB565 colors */
#define ST7735_BLACK       0x0000
#define ST7735_BLUE        0x001F
#define ST7735_RED         0xF800
#define ST7735_GREEN       0x07E0
#define ST7735_CYAN        0x07FF
#define ST7735_MAGENTA     0xF81F
#define ST7735_YELLOW      0xFFE0
#define ST7735_WHITE       0xFFFF
#define ST7735_ORANGE      0xFD20
#define ST7735_GRAY        0x8410

void ST7735_Init(void);
void ST7735_SetRotation(uint8_t m);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7735_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bg, uint8_t size);
void ST7735_WriteString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size);

#endif
