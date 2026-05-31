#include "radar.h"
#include "stdio.h"
#include "string.h"

ALIGN_32BYTES(static uint8_t rx_buf_left[RADAR_RX_BUF_SIZE])
__attribute__((section(".ARM.__at_0x24040000")));

ALIGN_32BYTES(static uint8_t rx_buf_right[RADAR_RX_BUF_SIZE])
__attribute__((section(".ARM.__at_0x24040100")));

static uint16_t read_ptr_left = 0;
static uint16_t read_ptr_right = 0;

static Radar_Data_t data_left = {0};
static Radar_Data_t data_right = {0};

// filter
#define RADAR_WINDOW_SIZE 30

bool is_scanning = false;

static uint8_t history_left[RADAR_WINDOW_SIZE];
static uint8_t history_right[RADAR_WINDOW_SIZE];

static uint16_t sum_left = 0;
static uint16_t sum_right = 0;

static uint16_t window_ptr = 0;

static uint32_t last_trigger_tick = 0;

void Radar_Start(void)
{
    HAL_UART_Receive_DMA(RADAR_UART_LEFT, rx_buf_left, RADAR_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(RADAR_UART_RIGHT, rx_buf_right, RADAR_RX_BUF_SIZE);

    __HAL_DMA_DISABLE_IT(RADAR_UART_LEFT->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(RADAR_UART_RIGHT->hdmarx, DMA_IT_HT);

    __HAL_DMA_DISABLE_IT(RADAR_UART_LEFT->hdmarx, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(RADAR_UART_RIGHT->hdmarx, DMA_IT_TC);

    memset(history_left, 0, sizeof(history_left));
    memset(history_right, 0, sizeof(history_right));
    sum_left = 0;
    sum_right = 0;
    window_ptr = 0;

    read_ptr_left = 0;
    read_ptr_right = 0;

    is_scanning = true;
}

void Radar_Stop(void)
{
    HAL_UART_DMAStop(RADAR_UART_LEFT);
    HAL_UART_DMAStop(RADAR_UART_RIGHT);

    is_scanning = false;

    memset(history_left, 0, sizeof(history_left));
    memset(history_right, 0, sizeof(history_right));
    sum_left = 0;
    sum_right = 0;
    window_ptr = 0;

    read_ptr_left = 0;
    read_ptr_right = 0;
}

/**
 * @brief  停止扫描，冻结投票数据
 */
void Radar_StopScanning(void)
{
    is_scanning = false;
}

static void Push_To_Window(bool is_left_detected, bool is_right_detected)
{
    // --- 处理左雷达窗口 ---
    // 1. 从总和中减去即将被覆盖掉的旧值
    sum_left -= history_left[window_ptr];
    // 2. 存入新值
    history_left[window_ptr] = is_left_detected ? 1 : 0;
    // 3. 将新值累加到总和
    sum_left += history_left[window_ptr];

    // --- 处理右雷达窗口 ---
    sum_right -= history_right[window_ptr];
    history_right[window_ptr] = is_right_detected ? 1 : 0;
    sum_right += history_right[window_ptr];

    // --- 移动指针 ---
    window_ptr++;
    if (window_ptr >= RADAR_WINDOW_SIZE)
    {
        window_ptr = 0; // 回绕
    }
}

static bool Parse_Radar_Buffer(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t *read_ptr, Radar_Data_t *output_data)
{
    bool found_new_frame = false;

    uint16_t write_ptr = RADAR_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    // 当读指针不等于写指针时，说明有新数据
    while (*read_ptr != write_ptr)
    {
        // 查找帧头 0x6E
        if (rx_buf[*read_ptr] == RADAR_FRAME_HEADER)
        {
            // 检查剩余数据量是否足够一帧 (5个字节)
            // 处理环形缓冲区的回绕
            uint16_t available_bytes = (write_ptr >= *read_ptr) ? (write_ptr - *read_ptr) : (RADAR_RX_BUF_SIZE - *read_ptr + write_ptr);

            if (available_bytes >= 5)
            {
                // 获取帧尾所在的索引位置 (考虑回绕)
                uint16_t tail_idx = (*read_ptr + 4) % RADAR_RX_BUF_SIZE;

                // 校验帧尾 0x62
                if (rx_buf[tail_idx] == RADAR_FRAME_TAIL)
                {

                    // 提取数据
                    uint8_t state = rx_buf[(*read_ptr + 1) % RADAR_RX_BUF_SIZE];
                    uint8_t dist_L = rx_buf[(*read_ptr + 2) % RADAR_RX_BUF_SIZE];
                    uint8_t dist_H = rx_buf[(*read_ptr + 3) % RADAR_RX_BUF_SIZE];

                    // 解析状态 (2或3表示有目标)
                    output_data->has_target = (state == 0x02 || state == 0x03);
                    // 解析距离 (小端模式)
                    output_data->distance = dist_L | (dist_H << 8);

                    // 读取完毕，指针直接跳过一帧
                    *read_ptr = (*read_ptr + 5) % RADAR_RX_BUF_SIZE;
                    found_new_frame = true;

                    continue;
                }
            }
        }

        // 如果当前字节不是帧头，或者数据不够，或者帧尾不对，读指针前移一字节继续找
        *read_ptr = (*read_ptr + 1) % RADAR_RX_BUF_SIZE;
    }

    return found_new_frame;
}

void Radar_Update(void)
{
    if (HAL_GetTick() - last_trigger_tick < 125) // 8hz
    {
        return;
    }
    last_trigger_tick = HAL_GetTick();

    if (RADAR_UART_LEFT->ErrorCode != HAL_UART_ERROR_NONE ||
        __HAL_UART_GET_FLAG(RADAR_UART_LEFT, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(RADAR_UART_LEFT, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
        HAL_UART_DMAStop(RADAR_UART_LEFT);
        HAL_UART_Receive_DMA(RADAR_UART_LEFT, rx_buf_left, RADAR_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(RADAR_UART_LEFT->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }

    if (RADAR_UART_RIGHT->ErrorCode != HAL_UART_ERROR_NONE ||
        __HAL_UART_GET_FLAG(RADAR_UART_RIGHT, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(RADAR_UART_RIGHT, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
        HAL_UART_DMAStop(RADAR_UART_RIGHT);
        HAL_UART_Receive_DMA(RADAR_UART_RIGHT, rx_buf_right, RADAR_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(RADAR_UART_RIGHT->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }

    // static uint32_t last_print_time = 0;
    // if (HAL_GetTick() - last_print_time > 500)
    // {
    //     //printf(".\r\n");
    //     last_print_time = HAL_GetTick();
    //     // --- 调试左雷达原始数据 ---
    //     uint16_t write_ptr_l = RADAR_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(RADAR_UART_LEFT->hdmarx);
    //     if (read_ptr_left != write_ptr_l)
    //     {
    //         printf("L:[ ");
    //         while (read_ptr_left != write_ptr_l)
    //         {
    //             printf("%02X ", rx_buf_left[read_ptr_left]);
    //             read_ptr_left = (read_ptr_left + 1) % RADAR_RX_BUF_SIZE;
    //         }
    //         printf("]\r\n");
    //     }

    //     // --- 调试右雷达原始数据 ---
    //     uint16_t write_ptr_r = RADAR_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(RADAR_UART_RIGHT->hdmarx);
    //     if (read_ptr_right != write_ptr_r)
    //     {
    //         printf("R:[ ");
    //         while (read_ptr_right != write_ptr_r)
    //         {
    //             printf("%02X ", rx_buf_right[read_ptr_right]);
    //             read_ptr_right = (read_ptr_right + 1) % RADAR_RX_BUF_SIZE;
    //         }
    //         printf("]\r\n");
    //     }
    //     //printf("\r\n[Radar] L:%d cm, R:%d cm\r\n", data_left.distance, data_right.distance);
    // }

    bool new_frame_L = Parse_Radar_Buffer(RADAR_UART_LEFT, rx_buf_left, &read_ptr_left, &data_left);
    bool new_frame_R = Parse_Radar_Buffer(RADAR_UART_RIGHT, rx_buf_right, &read_ptr_right, &data_right);

    if (is_scanning && (new_frame_L || new_frame_R))
    {
        bool left_obs = (data_left.has_target && data_left.distance < RADAR_DETECT_MAX_DIST);
        bool right_obs = (data_right.has_target && data_right.distance < RADAR_DETECT_MAX_DIST);

        Push_To_Window(left_obs, right_obs);
    }
}

Direction_t Radar_GetAvoidanceDirection(void)
{

    if (sum_left > (RADAR_WINDOW_SIZE * 0.4f))
    {
        return Direction_LEFT;
    }
    else
    {
        return Direction_RIGHT;
    }
}

// Direction_t Radar_GetAvoidanceDirection(void)
// {
//     if (sum_left > sum_right)
//     {
//         return Direction_RIGHT;
//     }
//     else if (sum_right > sum_left)
//     {
//         return Direction_LEFT;
//     }
//     else
//     {
//         return Direction_NORMAL;
//     }
// }

void Radar_DebugPrint(void)
{
    static uint32_t last_print_time = 0;

    if (HAL_GetTick() - last_print_time < 1000)
        return;
    last_print_time = HAL_GetTick();

    // 格式化输出：[状态] 距离 | 票数
    // L: [Y] 045cm | V:00123  R: [N] 000cm | V:00005 -> Choice: LEFT
    printf("Radar L:[%s] %3dcm, V:%u | ",
           data_left.has_target ? "Y" : "N",
           data_left.distance,
           sum_left);

    printf("R:[%s] %3dcm, V:%u",
           data_right.has_target ? "Y" : "N",
           data_right.distance,
           sum_right);

    Direction_t choice = Radar_GetAvoidanceDirection();
    const char *choice_str = (choice == Direction_LEFT) ? "TURN_LEFT" : (choice == Direction_RIGHT) ? "TURN_RIGHT"
                                                                                                    : "FORWARD";

    printf("\r\n=> Decision: %s\r\n", choice_str);
}