#include "st7735.h"
#include "font5x7.h"
#include "stm32h7xx_hal.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;


uint16_t ST7735_WIDTH  = ST7735_TFTWIDTH;
uint16_t ST7735_HEIGHT = ST7735_TFTHEIGHT;

#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_MADCTL  0x36
#define ST7735_COLMOD  0x3A

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_BGR 0x08

static uint8_t _rotation = 0;
static uint8_t _xstart = 0;
static uint8_t _ystart = 0;

static inline void TFT_Select(void)
{
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
}

static inline void TFT_Unselect(void)
{
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

static inline void TFT_DC_Command(void)
{
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
}

static inline void TFT_DC_Data(void)
{
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset(void)
{
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

static void ST7735_WriteCommand(uint8_t cmd)
{
    TFT_Select();
    TFT_DC_Command();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    TFT_Unselect();
}

static void ST7735_WriteData(uint8_t *data, uint16_t size)
{
    TFT_Select();
    TFT_DC_Data();
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    TFT_Unselect();
}

static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    x0 += _xstart;
    x1 += _xstart;
    y0 += _ystart;
    y1 += _ystart;

    ST7735_WriteCommand(ST7735_CASET);
    data[0] = 0x00;
    data[1] = x0;
    data[2] = 0x00;
    data[3] = x1;
    ST7735_WriteData(data, 4);

    ST7735_WriteCommand(ST7735_RASET);
    data[0] = 0x00;
    data[1] = y0;
    data[2] = 0x00;
    data[3] = y1;
    ST7735_WriteData(data, 4);

    ST7735_WriteCommand(ST7735_RAMWR);
}

static void ST7735_WriteColorBurst(uint16_t color, uint32_t len)
{
    uint8_t burst[128];
    for (uint16_t i = 0; i < 64; i++)
    {
        burst[2 * i]     = color >> 8;
        burst[2 * i + 1] = color & 0xFF;
    }

    TFT_Select();
    TFT_DC_Data();
    while (len)
    {
        uint16_t chunk = (len > 64) ? 64 : len;
        HAL_SPI_Transmit(&hspi1, burst, chunk * 2, HAL_MAX_DELAY);
        len -= chunk;
    }
    TFT_Unselect();
}

void ST7735_SetRotation(uint8_t m)
{
    _rotation = m % 4;
    ST7735_WriteCommand(ST7735_MADCTL);

    switch (_rotation)
    {
        case 0:
        {
            uint8_t madctl = MADCTL_MX | MADCTL_MY | MADCTL_BGR;
            ST7735_WriteData(&madctl, 1);
            ST7735_WIDTH = 128;
            ST7735_HEIGHT = 160;
            _xstart = 2;
            _ystart = 1;
        } break;

        case 1:
        {
            uint8_t madctl = MADCTL_MY | MADCTL_MV | MADCTL_BGR;
            ST7735_WriteData(&madctl, 1);
            ST7735_WIDTH = 160;
            ST7735_HEIGHT = 128;
            _xstart = 1;
            _ystart = 2;
        } break;

        case 2:
        {
            uint8_t madctl = MADCTL_BGR;
            ST7735_WriteData(&madctl, 1);
            ST7735_WIDTH = 128;
            ST7735_HEIGHT = 160;
            _xstart = 2;
            _ystart = 1;
        } break;

        case 3:
        {
            uint8_t madctl = MADCTL_MX | MADCTL_MV | MADCTL_BGR;
            ST7735_WriteData(&madctl, 1);
            ST7735_WIDTH = 160;
            ST7735_HEIGHT = 128;
            _xstart = 1;
            _ystart = 2;
        } break;
    }
}


void ST7735_Init(void)
{
    uint8_t data[16];

    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET);

    ST7735_Reset();

    ST7735_WriteCommand(ST7735_SWRESET);
    HAL_Delay(150);

    ST7735_WriteCommand(ST7735_SLPOUT);
    HAL_Delay(150);

    ST7735_WriteCommand(ST7735_FRMCTR1);
    data[0] = 0x01; data[1] = 0x2C; data[2] = 0x2D;
    ST7735_WriteData(data, 3);

    ST7735_WriteCommand(ST7735_FRMCTR2);
    data[0] = 0x01; data[1] = 0x2C; data[2] = 0x2D;
    ST7735_WriteData(data, 3);

    ST7735_WriteCommand(ST7735_FRMCTR3);
    data[0] = 0x01; data[1] = 0x2C; data[2] = 0x2D;
    data[3] = 0x01; data[4] = 0x2C; data[5] = 0x2D;
    ST7735_WriteData(data, 6);

    ST7735_WriteCommand(ST7735_INVCTR);
    data[0] = 0x07;
    ST7735_WriteData(data, 1);

    ST7735_WriteCommand(ST7735_PWCTR1);
    data[0] = 0xA2; data[1] = 0x02; data[2] = 0x84;
    ST7735_WriteData(data, 3);

    ST7735_WriteCommand(ST7735_PWCTR2);
    data[0] = 0xC5;
    ST7735_WriteData(data, 1);

    ST7735_WriteCommand(ST7735_PWCTR3);
    data[0] = 0x0A; data[1] = 0x00;
    ST7735_WriteData(data, 2);

    ST7735_WriteCommand(ST7735_PWCTR4);
    data[0] = 0x8A; data[1] = 0x2A;
    ST7735_WriteData(data, 2);

    ST7735_WriteCommand(ST7735_PWCTR5);
    data[0] = 0x8A; data[1] = 0xEE;
    ST7735_WriteData(data, 2);

    ST7735_WriteCommand(ST7735_VMCTR1);
    data[0] = 0x0E;
    ST7735_WriteData(data, 1);

    ST7735_WriteCommand(ST7735_INVOFF);

    ST7735_WriteCommand(ST7735_COLMOD);
    data[0] = 0x05;
    ST7735_WriteData(data, 1);

    ST7735_WriteCommand(ST7735_GMCTRP1);
    uint8_t gmctp[] = {0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
    ST7735_WriteData(gmctp, 16);

    ST7735_WriteCommand(ST7735_GMCTRN1);
    uint8_t gmctn[] = {0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
    ST7735_WriteData(gmctn, 16);

    ST7735_SetRotation(1);

    ST7735_WriteCommand(ST7735_NORON);
    HAL_Delay(10);

    ST7735_WriteCommand(ST7735_DISPON);
    HAL_Delay(100);

    ST7735_FillScreen(ST7735_BLACK);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;

    uint8_t data[2] = {color >> 8, color & 0xFF};
    ST7735_SetAddressWindow(x, y, x, y);
    ST7735_WriteData(data, 2);
}

void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if ((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if ((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    ST7735_WriteColorBurst(color, (uint32_t)w * h);
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    ST7735_FillRect(x, y, w, 1, color);
}

void ST7735_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    ST7735_FillRect(x, y, 1, h, color);
}

void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ST7735_DrawFastHLine(x, y, w, color);
    ST7735_DrawFastHLine(x, y + h - 1, w, color);
    ST7735_DrawFastVLine(x, y, h, color);
    ST7735_DrawFastVLine(x + w - 1, y, h, color);
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bg, uint8_t size)
{
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *bitmap = &Font5x7[(ch - 32) * 5];

    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t line = bitmap[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            uint16_t c = (line & 0x01) ? color : bg;
            if (size == 1)
                ST7735_DrawPixel(x + i, y + j, c);
            else
                ST7735_FillRect(x + i * size, y + j * size, size, size, c);
            line >>= 1;
        }
    }

    if (size == 1)
        ST7735_FillRect(x + 5, y, 1, 8, bg);
    else
        ST7735_FillRect(x + 5 * size, y, size, 8 * size, bg);
}

void ST7735_WriteString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size)
{
    while (*str)
    {
        if (x + 6 * size > ST7735_WIDTH)
        {
            x = 0;
            y += 8 * size;
        }
        if (y + 8 * size > ST7735_HEIGHT) break;

        ST7735_DrawChar(x, y, *str, color, bg, size);
        x += 6 * size;
        str++;
    }
}
