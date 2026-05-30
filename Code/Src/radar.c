#include "radar.h"
#include "stdio.h"

ALIGN_32BYTES(static uint8_t rx_buf_left[RADAR_RX_BUF_SIZE])
__attribute__((section(".ARM.__at_0x24040000")));

ALIGN_32BYTES(static uint8_t rx_buf_right[RADAR_RX_BUF_SIZE])
__attribute__((section(".ARM.__at_0x24040100")));

// 解析数据的读指针记录
static uint16_t read_ptr_left = 0;
static uint16_t read_ptr_right = 0;

// 解析出的最新雷达数据
static Radar_Data_t data_left = {0};
static Radar_Data_t data_right = {0};

// filter
static bool is_scanning = false;
static uint32_t left_obstacle_votes = 0;
static uint32_t right_obstacle_votes = 0;

/**
 * @brief  初始化雷达，开启 DMA 循环接收
 */
void Radar_Init(void)
{
    HAL_UART_Receive_DMA(RADAR_UART_LEFT, rx_buf_left, RADAR_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(RADAR_UART_RIGHT, rx_buf_right, RADAR_RX_BUF_SIZE);

    __HAL_DMA_DISABLE_IT(RADAR_UART_LEFT->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(RADAR_UART_RIGHT->hdmarx, DMA_IT_HT);

    __HAL_DMA_DISABLE_IT(RADAR_UART_LEFT->hdmarx, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(RADAR_UART_RIGHT->hdmarx, DMA_IT_TC);

    read_ptr_left = 0;
    read_ptr_right = 0;

    Radar_StartScanning(); // 默认可不开启，由主状态机调用
}

/**
 * @brief  开启扫描，清空之前的投票数据
 */
void Radar_StartScanning(void)
{
    left_obstacle_votes = 0;
    right_obstacle_votes = 0;
    is_scanning = true;
}

/**
 * @brief  停止扫描，冻结投票数据
 */
void Radar_StopScanning(void)
{
    is_scanning = false;
}

/**
 * @brief  私有函数：解析单个雷达的 DMA 环形缓冲区
 */
static void Parse_Radar_Buffer(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t *read_ptr, Radar_Data_t *output_data)
{
    // 获取当前 DMA 写指针位置 (RADAR_RX_BUF_SIZE 减去 剩余传输量)
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
                    continue;
                }
            }
        }

        // 如果当前字节不是帧头，或者数据不够，或者帧尾不对，读指针前移一字节继续找
        *read_ptr = (*read_ptr + 1) % RADAR_RX_BUF_SIZE;
    }
}

void Radar_Update(void)
{
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
    // if (HAL_GetTick() - last_print_time > 200)
    // {
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

    Parse_Radar_Buffer(RADAR_UART_LEFT, rx_buf_left, &read_ptr_left, &data_left);
    Parse_Radar_Buffer(RADAR_UART_RIGHT, rx_buf_right, &read_ptr_right, &data_right);

    if (is_scanning)
    {
        // 左侧雷达检测到目标且距离在阈值内
        if (data_left.has_target && data_left.distance > 0 && data_left.distance < RADAR_DETECT_MAX_DIST)
        {
            left_obstacle_votes++;
        }

        // 右侧雷达检测到目标且距离在阈值内
        if (data_right.has_target && data_right.distance > 0 && data_right.distance < RADAR_DETECT_MAX_DIST)
        {
            right_obstacle_votes++;
        }
    }
}

Direction_t Radar_GetAvoidanceDirection(void)
{
    if (left_obstacle_votes > right_obstacle_votes)
    {
        return Direction_RIGHT;
    }

    else if (right_obstacle_votes > left_obstacle_votes)
    {
        return Direction_LEFT;
    }
    else
    {
        return Direction_NORMAL;
    }
}

void Radar_DebugPrint(void)
{
    static uint32_t last_print_time = 0;

    // 限制打印频率为 5Hz (200ms 一次)，防止占用过多串口带宽
    if (HAL_GetTick() - last_print_time < 200)
        return;
    last_print_time = HAL_GetTick();

    // 格式化输出：[状态] 距离 | 票数
    // L: [Y] 045cm | V:00123  R: [N] 000cm | V:00005 -> Choice: LEFT
    printf("Radar L:[%s] %3dcm, V:%u | ",
           data_left.has_target ? "Y" : "N",
           data_left.distance,
           left_obstacle_votes);

    printf("R:[%s] %3dcm, V:%u",
           data_right.has_target ? "Y" : "N",
           data_right.distance,
           right_obstacle_votes);

    Direction_t choice = Radar_GetAvoidanceDirection();
    const char *choice_str = (choice == Direction_LEFT) ? "TURN_LEFT" : (choice == Direction_RIGHT) ? "TURN_RIGHT"
                                                                                                    : "FORWARD";

    printf("\r\n=> Decision: %s\r\n", choice_str);
}