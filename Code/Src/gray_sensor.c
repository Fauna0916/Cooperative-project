#include "gray_sensor.h"
#include <stdlib.h>
#include <math.h>

// ========== 配置参数 ==========
#define MAX_ERR_STEP 25    // 每帧允许的最大偏差变化量（防跳变核心）
#define LOST_THRESHOLD 2000
#define LEFT_ERR (-100)
#define RIGHT_ERR (100)

// ========== 静态变量 ==========
static uint32_t lost_frame_cnt = 0;
static int16_t last_valid_err_f = 0;
static uint8_t force_update_flag = 0; // 强制更新标志位
GraySensor_Data_t gray_data = {0};


typedef struct
{
    int16_t center_err;
    uint8_t width;
    uint8_t is_left;
    uint8_t is_right;
} LineBlob_t;

static inline void Multiplexer_Delay(void)
{
    for (volatile int i = 0; i < 15; i++)
    {
        __NOP();
    }
}

static inline uint8_t count_directions(uint8_t dir_avail)
{
    uint8_t cnt = 0;
    if (dir_avail & 0x01)
        cnt++;
    if (dir_avail & 0x02)
        cnt++;
    if (dir_avail & 0x04)
        cnt++;
    return cnt;
}

void GraySensor_Init(void)
{
    last_valid_err_f = 0;
    force_update_flag = 0;
}

GraySensor_Data_t *GraySensor_GetData(void)
{
    return &gray_data;
}

/**
 * @brief 外部调用：强制设定偏差参考点（如转弯决策时刻调用）
 */
void GraySensor_ForceSetLastErr(int16_t forced_err)
{
    last_valid_err_f = forced_err;
    force_update_flag = 1; // 标记下一次 Update 不需要限速
}

static uint8_t GraySensor_ReadRaw(void)
{
    uint8_t raw = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (i & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (i & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        Multiplexer_Delay();
        if (HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin))
        {
            raw |= (1 << (7 - i)); // Bit7最左, Bit0最右
        }
    }
    return raw;
}

void GraySensor_Update(void)
{
    uint8_t current_raw = GraySensor_ReadRaw();
    gray_data.raw_data = current_raw;

    // 1. 丢线处理
    if (current_raw == 0x00)
    {
        if (++lost_frame_cnt < LOST_THRESHOLD)
        {
            gray_data.flag = GraySensor_FLAG_NORMAL;
            gray_data.err_f = last_valid_err_f;
        }
        else
        {
            gray_data.flag = GraySensor_FLAG_LOST;
            gray_data.err_f = (last_valid_err_f < 0) ? -100 : 100;
        }
        gray_data.err_l = LEFT_ERR;
        gray_data.err_r = RIGHT_ERR;
        return;
    }
    lost_frame_cnt = 0;

    // 2. 连通域分析
    LineBlob_t blobs[4];
    uint8_t blob_count = 0;
    int current_blob_start = -1;

    for (int8_t i = 7; i >= -1; i--)
    {
        uint8_t bit_is_1 = (i >= 0) ? ((current_raw >> i) & 0x01) : 0;
        if (bit_is_1)
        {
            if (current_blob_start == -1)
                current_blob_start = i;
        }
        else if (current_blob_start != -1)
        {
            uint8_t end = i + 1;
            float center_bit = (current_blob_start + end) / 2.0f;
            float dist_from_center = center_bit - 3.5f;
            float abs_dist = fabsf(dist_from_center);

            // 非线性映射
            float scaled_err = (abs_dist <= 1.5f) ? (abs_dist * 26.6f) : (40.0f + (abs_dist - 1.5f) * 30.0f);
            int16_t err = (int16_t)scaled_err;
            if (dist_from_center > 0)
                err = -err; // 左偏为负, 右偏为正 (根据你之前的逻辑)

            blobs[blob_count].center_err = err;
            blobs[blob_count].width = current_blob_start - end + 1;
            blobs[blob_count].is_left = (current_blob_start >= 5);
            blobs[blob_count].is_right = (end <= 2);
            blob_count++;
            current_blob_start = -1;
        }
    }

    // 3. 目标选择（寻找最接近上次偏差的块）
    int16_t target_err = blobs[0].center_err;
    uint8_t best_idx = 0;
    if (blob_count > 1)
    {
        int16_t min_diff = 999;
        for (uint8_t i = 0; i < blob_count; i++)
        {
            int16_t diff = abs(blobs[i].center_err - last_valid_err_f);
            if (diff < min_diff)
            {
                min_diff = diff;
                best_idx = i;
            }
        }
        target_err = blobs[best_idx].center_err;
    }

    // 4. 【核心变化率限制逻辑】
    if (force_update_flag)
    {
        // 如果外部强制设定了位置，直接同步，不进行平滑
        last_valid_err_f = target_err;
        force_update_flag = 0;
    }
    else
    {
        // 正常行驶模式：限制每帧的变化步长，防止干扰跳变
        int16_t delta = target_err - last_valid_err_f;
        if (delta > MAX_ERR_STEP)
            delta = MAX_ERR_STEP;
        if (delta < -MAX_ERR_STEP)
            delta = -MAX_ERR_STEP;
        last_valid_err_f += delta;
    }

    // 5. 更新输出
    gray_data.err_f = last_valid_err_f;

    // err_l/r 逻辑优化
    gray_data.err_l = (blobs[best_idx].center_err < -20) ? blobs[best_idx].center_err : LEFT_ERR;
    gray_data.err_r = (blobs[best_idx].center_err > 20) ? blobs[best_idx].center_err : RIGHT_ERR;

    // 6. 路口判定
    uint8_t dir_avail = 0;
    if (blobs[best_idx].width >= 4)
    {
        if (blobs[best_idx].is_left)
            dir_avail |= 0x01;
        if (abs(blobs[best_idx].center_err) < 45)
            dir_avail |= 0x02;
        if (blobs[best_idx].is_right)
            dir_avail |= 0x04;
        if (blobs[best_idx].width >= 6)
            dir_avail = 0x07;
    }

    // 5. 路口判定：只要左侧(0x01)或右侧(0x04)有信号，就视为路口
    // 逻辑：dir_avail & 0x05 (即 0x01 | 0x04)
    if (count_directions(dir_avail) >= 2)
    {
        gray_data.flag = GraySensor_FLAG_JUNC | (dir_avail & 0x0F);
    }
    else
    {
        gray_data.flag = GraySensor_FLAG_NORMAL;
    }
}