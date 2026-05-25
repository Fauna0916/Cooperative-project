#include "gray_sensor.h"

#define LOST_THRESHOLD 2000 // 丢线容忍帧数。假设1kHz采样，50帧等于50ms。根据车速调整。
static uint16_t lost_frame_cnt = 0;
GraySensor_Data_t gray_data = {0};
typedef struct
{
    int16_t center_err; // 该黑块的中心偏差 (-100 到 100)
    uint8_t width;      // 该黑块包含的传感器数量
    uint8_t is_left;    // 是否靠近左侧
    uint8_t is_right;   // 是否靠近右侧
} LineBlob_t;

// 内部静态变量记录上一次的偏差，用于防干扰追踪和脱线找回
static int16_t last_valid_err_f = 0;

static inline void Multiplexer_Delay(void)
{
    // 74HC4051 切换延迟约 20-50ns。400MHz下1个NOP是2.5ns
    // 循环或NOP指令确保ADC/GPIO读取前电平已稳定
    for (volatile int i = 0; i < 15; i++)
    {
        __NOP();
    }
}

void GraySensor_Init(void)
{
    last_valid_err_f = 0;
}

GraySensor_Data_t *GraySensor_GetData(void)
{
    return &gray_data;
}



/**
 * @brief 底层读取8路复用传感器原始数据
 * @return 8位无符号整数，Bit7对应最左侧传感器，Bit0对应最右侧
 */
static uint8_t GraySensor_ReadRaw(void)
{
    uint8_t raw = 0;

    // 遍历通道 0 到 7
    for (uint8_t i = 0; i < 8; i++)
    {
        // 设置 AD0, AD1, AD2
        HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (i & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (i & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        Multiplexer_Delay(); // 关键延时，防止读入虚假电平

        uint8_t bit_val = HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin);

// 统一转换为：1表示踩到黑线，0表示白色背景
#if SENSOR_BLACK_ACTIVE_LEVEL == 0
        bit_val = !bit_val;
#endif

        if (bit_val)
        {
            // 假设通道 1 (i=0) 是最右侧(Bit0)，通道 8 (i=7) 是最左侧(Bit7)
            // 如果您的硬件安装相反，可改为 raw |= (1 << (7 - i));
            raw |= (1 << i);
        }
    }
    return raw;
}

/**
 * @brief 核心更新函数：读取数据，分离线段，计算偏差，识别路口
 * @param gray_data 传感器数据结构体指针
 */
void GraySensor_Update(void)
{

    uint8_t current_raw = GraySensor_ReadRaw();
    gray_data.raw_data = current_raw;

    // 1. 脱线/丢线逻辑 (引入缓冲机制)
    if (current_raw == 0x00)
    {
        lost_frame_cnt++;

        if (lost_frame_cnt < LOST_THRESHOLD)
        {
            // --- 【阶段 A：丢线缓冲期】 ---
            // 伪装成 NORMAL 状态，让主状态机继续运行巡线逻辑
            gray_data.flag = GraySensor_FLAG_NORMAL;
            // 保持最后的有效偏差，实现“惯性巡线”
            gray_data.err_f = last_valid_err_f;

            // 预设转弯偏差（以防主状态机在缓冲期内进路口逻辑）
            gray_data.err_l = 100;
            gray_data.err_r = -100;
        }
        else
        {
            // --- 【阶段 B：正式确认丢线】 ---
            // 超过阈值，触发真正的 LOST 状态
            gray_data.flag = GraySensor_FLAG_LOST;
            // 这里的 err_f 用于搜线方向引导：往最后一次偏离的方向找
            gray_data.err_f = (last_valid_err_f > 0) ? 100 : -100;
            gray_data.err_l = 100;
            gray_data.err_r = -100;
        }
        return; 
    }
    else
    {
        lost_frame_cnt = 0;
    }

    // 2. 连通域分析 (Blob Extraction) —— 解决1.1平行线干扰的核心
    LineBlob_t blobs[4];
    uint8_t blob_count = 0;
    int current_blob_start = -1;

    for (int8_t i = 7; i >= -1; i--) // 从左到右遍历位，多扫描一次-1用于收尾
    {
        uint8_t bit_is_1 = (i >= 0) ? ((gray_data.raw_data >> i) & 0x01) : 0;

        if (bit_is_1)
        {
            if (current_blob_start == -1)
            {
                current_blob_start = i; // 发现新黑块起点
            }
        }
        else
        {
            if (current_blob_start != -1)
            {
                // 结算当前黑块
                uint8_t end = i + 1;
                uint8_t width = current_blob_start - end + 1;

                // 计算质心位置 (0~7)，转换为 -100(最右) 到 100(最左) 的偏差
                // bit7对应+100, bit0对应-100。公式：err = (center_bit - 3.5) * (200 / 7)
                float center_bit = (current_blob_start + end) / 2.0f;
                int16_t err = (int16_t)((center_bit - 3.5f) * 28.57f);

                blobs[blob_count].center_err = err;
                blobs[blob_count].width = width;
                blobs[blob_count].is_left = (current_blob_start >= 5); // 触及左侧边缘
                blobs[blob_count].is_right = (end <= 2);               // 触及右侧边缘

                blob_count++;
                current_blob_start = -1;
            }
        }
    }

    // 3. 偏差分配与抗干扰逻辑
    int16_t best_err_f = 0;
    int min_diff = 999;
    uint8_t dir_avail = 0; // 记录路口可用方向: bit0=左, bit1=前, bit2=右

    for (uint8_t i = 0; i < blob_count; i++)
    {
        // 寻找与上一次 err_f 最接近的 Blob 作为当前的前进线 (无视突然出现的旁线干扰)
        int diff = abs(blobs[i].center_err - last_valid_err_f);
        if (diff < min_diff)
        {
            min_diff = diff;
            best_err_f = blobs[i].center_err;
        }

        // 判断路口分支
        if (blobs[i].is_left)
            dir_avail |= 0x01; // 左侧有线
        if (abs(blobs[i].center_err) < 40)
            dir_avail |= 0x02; // 中间有线
        if (blobs[i].is_right)
            dir_avail |= 0x04; // 右侧有线

        // 如果遇到单个极宽的黑块 (例如丁字路口、十字路口)
        if (blobs[i].width >= 5)
        {
            dir_avail |= 0x07; // 默认所有方向均可能
        }
    }

    // 更新数据
    gray_data.err_f = best_err_f;
    last_valid_err_f = best_err_f; // 保存有效值

    // 默认提供强制转弯的极端偏差，当决策函数决定转弯时，PID使用这些值
    gray_data.err_l = 100;  // 强行偏左
    gray_data.err_r = -100; // 强行偏右

    // 4. 判定是否为路口 (解决地图1.3相切问题)
    // 如果不仅仅是只有中间一条线，则认为是路口/分支
    if (dir_avail > 0x02 || blob_count > 1)
    {
        // 组合状态: 高4位是JUNC标志，低4位是可用方向
        gray_data.flag = GraySensor_FLAG_JUNC | (dir_avail & 0x0F);
    }
    else
    {
        gray_data.flag = GraySensor_FLAG_NORMAL;
    }
}
