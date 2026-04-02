#include "st7735.h"

extern SPI_HandleTypeDef hspi4;

// ==================== 引脚控制宏 ====================
#define ST7735_CS_LOW()    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)
#define ST7735_CS_HIGH()   HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)

#define ST7735_DC_LOW()    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET)
#define ST7735_DC_HIGH()   HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET)

// 背光低电平点亮
#define ST7735_BL_ON()     HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_RESET)
#define ST7735_BL_OFF()    HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET)


// ==================== 基础函数 ====================
static void ST7735_WriteCommand(uint8_t cmd)
{
    ST7735_CS_LOW();
    ST7735_DC_LOW();
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    ST7735_CS_HIGH();
}

static void ST7735_WriteData(uint8_t *data, uint16_t size)
{
    ST7735_CS_LOW();
    ST7735_DC_HIGH();
    HAL_SPI_Transmit(&hspi4, data, size, 100);
    ST7735_CS_HIGH();
}

static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    x0 += ST7735_XSTART;
    x1 += ST7735_XSTART;
    y0 += ST7735_YSTART;
    y1 += ST7735_YSTART;

    // Column address set
    ST7735_WriteCommand(0x2A);
    data[0] = 0x00;
    data[1] = x0;
    data[2] = 0x00;
    data[3] = x1;
    ST7735_WriteData(data, 4);

    // Row address set
    ST7735_WriteCommand(0x2B);
    data[0] = 0x00;
    data[1] = y0;
    data[2] = 0x00;
    data[3] = y1;
    ST7735_WriteData(data, 4);

    // RAM write
    ST7735_WriteCommand(0x2C);
}

// ==================== 初始化 ====================
void ST7735_Init(void)
{
    uint8_t data;

    // 先强制开背光，方便确认背光逻辑
    ST7735_BL_ON();
    HAL_Delay(200);

    // 软件复位
    ST7735_WriteCommand(0x01);
    HAL_Delay(150);

    // 退出睡眠
    ST7735_WriteCommand(0x11);
    HAL_Delay(120);

    // 16-bit RGB565
    ST7735_WriteCommand(0x3A);
    data = 0x05;
    ST7735_WriteData(&data, 1);
    HAL_Delay(10);

    // 扫描方向
    ST7735_WriteCommand(0x36);
    data = 0x08;   // 如果方向不对可以试 0xC8
    ST7735_WriteData(&data, 1);
    HAL_Delay(10);

    // 反色（很多 IPS 小屏需要）
    ST7735_WriteCommand(0x21);
    HAL_Delay(10);

    // 开显示
    ST7735_WriteCommand(0x29);
    HAL_Delay(50);

    ST7735_BL_ON();
}

// ==================== 画点 ====================
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t data[2];

    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT)
        return;

    ST7735_SetAddressWindow(x, y, x, y);

    data[0] = (uint8_t)(color >> 8);
    data[1] = (uint8_t)(color & 0xFF);
    ST7735_WriteData(data, 2);
}

// ==================== 全屏填充 ====================
void ST7735_FillScreen(uint16_t color)
{
    uint32_t totalPixels = ST7735_WIDTH * ST7735_HEIGHT;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    // 一次发 1024 个像素 = 2048 字节
    static uint8_t buf[2048];
    for (uint16_t i = 0; i < sizeof(buf); i += 2)
    {
        buf[i] = hi;
        buf[i + 1] = lo;
    }

    ST7735_SetAddressWindow(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);

    ST7735_CS_LOW();
    ST7735_DC_HIGH();

    while (totalPixels > 0)
    {
        uint16_t pixelsThisTime = (totalPixels > 1024) ? 1024 : totalPixels;
        HAL_SPI_Transmit(&hspi4, buf, pixelsThisTime * 2, HAL_MAX_DELAY);
        totalPixels -= pixelsThisTime;
    }

    ST7735_CS_HIGH();
}


// ==================== 填充矩形 ====================
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if ((x + w) > ST7735_WIDTH)  w = ST7735_WIDTH - x;
    if ((y + h) > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    uint32_t totalPixels = w * h;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    static uint8_t buf[2048];
    for (uint16_t i = 0; i < sizeof(buf); i += 2)
    {
        buf[i] = hi;
        buf[i + 1] = lo;
    }

    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    ST7735_CS_LOW();
    ST7735_DC_HIGH();

    while (totalPixels > 0)
    {
        uint16_t pixelsThisTime = (totalPixels > 1024) ? 1024 : totalPixels;
        HAL_SPI_Transmit(&hspi4, buf, pixelsThisTime * 2, HAL_MAX_DELAY);
        totalPixels -= pixelsThisTime;
    }

    ST7735_CS_HIGH();
}

