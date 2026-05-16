#include "gray_sensor.h"

#define SOFT_LOST_TIMEOUT 30 // 软丢线帧数。在 500Hz 下，30帧约为 60ms
static uint32_t lost_counter = 0; 

GraySensor_Data_t gray_data = {0};

// --- 配置区 ---
// 假设黑线输出 1，白底输出 0。
#define BLACK_LINE_ACTIVE_STATE GPIO_PIN_SET

// 【优化 1】：非线性权重 (应对 1.1 难点)
// 中间极小(2)，保证直行不发抖；外侧极大(100)，保证波浪急弯死死咬住线。
static const int16_t GraySensor_WEIGHTS[8] = {-100, -50, -15, -2, 2, 15, 50, 100};

static int16_t last_error = 0;

static inline void Set_Mux_Channel(uint8_t ch)
{
    // 根据你的真值表 000 -> CH1, 111 -> CH8
    HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline uint8_t Read_Sensor_Out(void)
{
    return (HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin) == BLACK_LINE_ACTIVE_STATE) ? 1 : 0;
}

// 基于主频的微秒级延时，应对 400MHz 芯片
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
 * @brief  传感器核心处理逻辑 (建议放入 500Hz~1000Hz 定时器)
 */
void GraySensor_Update(void)
{
    uint8_t raw_mask = 0;

    // 1. 硬件扫描 8 个通道获取数据 (Bit0->CH1, Bit7->CH8)
    for (uint8_t i = 0; i < 8; i++)
    {
        Set_Mux_Channel(i);
        Delay_us(2); 

        if (Read_Sensor_Out() == 1)
        {
            raw_mask |= (1 << i);
        }
    }
    gray_data.raw_data = raw_mask;

    // --- 提取区域特征 ---
    bool is_left_on = (raw_mask & 0x03);   // CH1, CH2
    bool is_center_on = (raw_mask & 0x18); // CH4, CH5
    bool is_right_on = (raw_mask & 0xC0);  // CH7, CH8

    // 2. 计算巡线误差与抗干扰逻辑
    int32_t sum_weight = 0;
    uint8_t error_active_count = 0;
    uint8_t total_active_count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (raw_mask & (1 << i))
        {
            total_active_count++;

            // 【优化 2】：中心锁定机制 (应对 1.3 难点，防止被切线圆吸入)
            // 如果中间在车道内，且触发的是最边缘(i=0 或 i=7)，则剥夺其对转向误差的贡献！
            if (is_center_on && (i == 0 || i == 7))
            {
                continue;
            }

            sum_weight += GraySensor_WEIGHTS[i];
            error_active_count++;
        }
    }

    if (total_active_count > 0)
    {
        lost_counter = 0;

        if (total_active_count >= 5 || (is_left_on && is_right_on))
        {
            // --- 情况 B: 十字路口 / 丁字路 / 大面积干扰 ---
            // 判断条件强化：触发大于5个，或者左右两端同时触发(中间断开的情况)
            gray_data.flag = GraySensor_FLAG_JUNC;

            // 构造方向可用掩码，无缝对接 Decide_Shortest_Path
            if (is_left_on)
                gray_data.flag |= 0x01; // Bit0: 左边有路
            if (is_center_on)
                gray_data.flag |= 0x02; // Bit1: 前方有路
            if (is_right_on)
                gray_data.flag |= 0x04; // Bit2: 右边有路

            // 设置各个方向的目标靶点 (为你后续的分段转向PID留好接口)
            gray_data.err_l = GraySensor_WEIGHTS[1]; // 左转靶点
            gray_data.err_r = GraySensor_WEIGHTS[6]; // 右转靶点
            gray_data.err_f = 0;                     // 直行靶点

            // 路口期间，仍维持正常的误差输出，防止小车在路口发飙
            if (error_active_count > 0)
            {
                gray_data.current_error = sum_weight / error_active_count;
            }
            else
            {
                gray_data.current_error = last_error;
            }
        }
        else
        {
            gray_data.flag = GraySensor_FLAG_NORMAL;
        }

        // 正常计算误差并更新记忆
        gray_data.current_error = sum_weight / error_active_count;
        last_error = gray_data.current_error;
    }
    else
    {
        // --- 关键修改：进入丢线检测逻辑 ---
        lost_counter++;

        if (lost_counter < SOFT_LOST_TIMEOUT)
        {
            /* 【软丢线阶段】：不报 LOST，不触发主状态机原地搜线 */
            // 我们依然报 NORMAL，但通过 current_error 输出一个极大的转向修正
            // 引导 PID 环路在惯性作用下把车头“甩”回线上
            gray_data.flag = GraySensor_FLAG_NORMAL;

            // 维持最后一次偏航的极值误差
            gray_data.current_error = (last_error > 0) ? 110 : -110;
        }
        else
        {
            /* 【硬丢线阶段】：正式报 LOST，交由主状态机接管 */
            // 此时说明小车已经完全脱离轨道且自动纠偏失败
            gray_data.flag = GraySensor_FLAG_LOST;
            gray_data.current_error = 0; // 或者保持极值，取决于搜线策略
        }
    }
}

GraySensor_Data_t *GraySensor_GetData(void)
{
    return &gray_data;
}