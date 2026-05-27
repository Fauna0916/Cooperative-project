#include "gray_sensor.h"
#include "stdlib.h"

// PID 逻辑是“正数向右转  err_f 应该为正

#define LOST_THRESHOLD 2000 // 丢线容忍帧数。假设1kHz采样，50帧等于50ms。根据车速调整。
static uint16_t lost_frame_cnt = 0;
GraySensor_Data_t gray_data = {0};

#define SENSOR_BLACK_ACTIVE_LEVEL (1)
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
    // 切换延迟约 20-50ns。400MHz下1个NOP是2.5ns
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
 * @return 8位无符号整数，Bit0对应最左侧传感器，Bit7对应最右侧
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
            // 通道1(i=0) -> Bit 7 (最左)
            // 通道8(i=7) -> Bit 0 (最右)
            raw |= (1 << (7 - i));
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

                static const float weight_table[] = {100.0f, 75.0f, 40.0f, 15.0f};
                float center_bit = (current_blob_start + end) / 2.0f;
                float dist_from_center = 3.5f - center_bit; // 范围 -3.5 到 +3.5

                int16_t err = 0;
                float abs_dist = fabs(dist_from_center);

                // --- 非线性权重分配 ---
                if (abs_dist < 0.1f)
                    err = 0; // 【修正】绝对中心，不纠偏
                else if (abs_dist <= 0.5f)
                    err = 15; // 微小纠偏
                else if (abs_dist <= 1.5f)
                    err = 40; // 中度纠偏
                else if (abs_dist <= 2.5f)
                    err = 75; // 强力纠偏
                else
                    err = 100; // 极限救车

                // 恢复符号
                if (dist_from_center < 0)
                    err = -err;

                blobs[blob_count].center_err = err;
                blobs[blob_count].width = width;
                blobs[blob_count].is_left = (current_blob_start >= 5); // 触及左侧边缘
                blobs[blob_count].is_right = (end <= 2);               // 触及右侧边缘

                blob_count++;
                current_blob_start = -1;
            }
        }
    }

    // --- 3. 偏差分配与动态方向解析 ---
    int16_t best_err_f = 0;
    int16_t best_err_l = 100; // 默认值，防止没找到块时失效
    int16_t best_err_r = -100;

    int min_diff_f = 999;
    uint8_t dir_avail = 0;

    for (uint8_t i = 0; i < blob_count; i++)
    {
        // A. 寻找最接近上一次偏差的块作为前进线 (err_f)
        int diff = abs(blobs[i].center_err - last_valid_err_f);
        if (diff < min_diff_f)
        {
            min_diff_f = diff;
            best_err_f = blobs[i].center_err;
        }

        // B. 寻找最靠左的块作为左转引导线 (err_l)
        // 只有当这个块在中心左侧或触及左边缘时才考虑
        if (blobs[i].center_err > 10) // TODO: ensure threshold
        {
            // 找出最左（偏差正值最大）的块
            if (blobs[i].center_err > best_err_l || best_err_l == 100)
            {
                best_err_l = blobs[i].center_err;
            }
        }

        // C. 寻找最靠右的块作为右转引导线 (err_r)
        if (blobs[i].center_err < -10)
        {
            // 找出最右（偏差负值绝对值最大）的块
            if (blobs[i].center_err < best_err_r || best_err_r == -100)
            {
                best_err_r = blobs[i].center_err;
            }
        }

        // D. 路口标志位判定
        if (blobs[i].is_left)
            dir_avail |= 0x01;
        if (abs(blobs[i].center_err) < 40)
            dir_avail |= 0x02;
        if (blobs[i].is_right)
            dir_avail |= 0x04;

        if (blobs[i].width >= 5)
            dir_avail |= 0x07;
    }

    // 更新输出数据
    gray_data.err_f = best_err_f;
    last_valid_err_f = best_err_f;

    // 现在 err_l 和 err_r 是真实反映左/右分支线位置的动态值了
    gray_data.err_l = best_err_l;
    gray_data.err_r = best_err_r;

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
