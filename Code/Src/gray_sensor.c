#include "gray_sensor.h"

GraySensor_Data_t gray_data = {0};

// --- 配置区 ---
// 假设黑线输出 1，白底输出 0。如果是反的，请改为 0
#define BLACK_LINE_ACTIVE_STATE GPIO_PIN_SET

// 探头的物理横向权重 (从左到右: CH1 到 CH8)
static const int16_t GraySensor_WEIGHTS[8] = {-70, -50, -30, -10, 10, 30, 50, 70};

// 短时记忆
static int16_t last_error = 0;

static inline void Set_Mux_Channel(uint8_t ch)
{
    // 对应你图表中的 AD2, AD1, AD0
    HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline uint8_t Read_Sensor_Out(void)
{
    return (HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin) == BLACK_LINE_ACTIVE_STATE) ? 1 : 0;
}

// 微秒级延时，等待多路复用器芯片(74HC4051)电平稳定
static inline void Delay_us(uint32_t us)
{
    uint32_t count = us * (SystemCoreClock / 1000000) / 4;
    while (count--)
    {
        __NOP();
    }
}

void GraySensor_Init(void)
{
    gray_data.flag = GraySensor_FLAG_LOST;
}

/**
 * @brief  在定时器中断或主循环中高频调用 (建议 500Hz ~ 1000Hz)
 */
void GraySensor_Update(void)
{
    uint8_t raw_mask = 0;
    int32_t sum_weight = 0;
    uint8_t active_count = 0;

    // 1. 扫描 8 个通道获取数据
    // 注意：根据你的真值表，000(0)->CH1, 111(7)->CH8
    for (uint8_t i = 0; i < 8; i++)
    {
        Set_Mux_Channel(i);
        Delay_us(2); // 等待引脚电平稳定

        if (Read_Sensor_Out() == 1)
        {
            raw_mask |= (1 << i); 
            sum_weight += GraySensor_WEIGHTS[i];
            active_count++;
        }
    }

    gray_data.raw_data = raw_mask;

    // 2. 状态机判断与路况解析
    if (active_count == 0)
    {
        // --- 情况 A: 丢线 (Lost) ---
        gray_data.flag = GraySensor_FLAG_LOST;
        // 使用最后的记忆进行盲搜（满舵）
        gray_data.current_error = (last_error > 0) ? 80 : -80;
    }
    else if (active_count >= 5 || (raw_mask == 0x81))
    {
        // --- 情况 B: 十字路口 / 90度直角 ---
        // 如果触发了 5 个以上的探头(横跨黑线)，或者最左和最右同时触发(0x81, 可能是中间断开的十字)
        gray_data.flag = GraySensor_FLAG_JUNC;

        // 构造岔路口可用掩码 (为了无缝接入 Decide_Shortest_Path)
        // Bit 0: 左侧有路 (CH1, CH2 触发)
        if (raw_mask & 0x03)
            gray_data.flag |= 0x01;

        // Bit 1: 前方有路 (CH4, CH5 触发)
        if (raw_mask & 0x18)
            gray_data.flag |= 0x02;

        // Bit 2: 右侧有路 (CH7, CH8 触发)
        if (raw_mask & 0xC0)
            gray_data.flag |= 0x04;

        // 设置各个方向的目标靶点
        gray_data.err_l = -80; // 如果选左转，PID 靶点在最左边
        gray_data.err_r = 80;  // 如果选右转，PID 靶点在最右边
        gray_data.err_f = 0;   // 如果选直行，PID 靶点在正中央

        // 保留当前正常误差，防止算法还没做决策时失去控制
        gray_data.current_error = sum_weight / active_count;
    }
    else
    {
        // --- 情况 C: 正常巡线 / 波浪线 / 普通弯道 ---
        gray_data.flag = GraySensor_FLAG_NORMAL;
        gray_data.current_error = sum_weight / active_count;
        last_error = gray_data.current_error; // 刷新记忆
    }
}

GraySensor_Data_t *GraySensor_GetData(void)
{
    return &gray_data;
}