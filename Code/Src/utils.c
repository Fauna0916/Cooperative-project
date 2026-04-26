#include "utils.h"
#include "string.h"
#include "odometry.h"

uint8_t debug_flag = 0;

#define RX_BUF_SIZE 100

ALIGN_32BYTES(uint8_t rx_buffer[RX_BUF_SIZE])
__attribute__((section(".ARM.__at_0x24000040")));

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

void I2C_VerifyCommunication(I2C_HandleTypeDef *device_I2C, uint16_t device_address)
{
    uint8_t buf[1] = {1};
    if (HAL_I2C_Master_Transmit(device_I2C, device_address, buf, 1, 100) == HAL_OK)
    {
        printf("Success\r\n");
    }
    else
    {
        printf("Hardware Not Found!\r\n");
    }
}

void debug_info(void)
{
    if (debug_flag)
    {
        debug_flag = 0;
        // printf("letf:%d\r\n", Encoder_GetLeftData()->speed_rpm);
        // printf("right:%d\r\n", Encoder_GetRightData()->speed_rpm);
        // printf("x:%.1f,y:%.1f,theta:%.1f\r\n", Odometry_GetState()->x, Odometry_GetState()->y, Odometry_GetState()->theta);
        // printf("[Enc] v:%.3f,w:%.3f\r\n", Encoder_GetLinearVelocity(), Encoder_GetAngularVelocity());
        //printf("[Odo] v:%.3f,w:%.3f\r\n", Odometry_GetState()->linear_vel, Odometry_GetState()->angular_vel);
        printf("YPR: %.1f %.1f %.1f\r\n", BNO080_GetLatestData()->yaw * 57.29578f,
               BNO080_GetLatestData()->pitch * 57.29578f,
               BNO080_GetLatestData()->roll * 57.29578f);
    }
}

void Tuning_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    printf("PID Debug Ready! Format: {kp,ki,kd@}\r\n");
}

void Parse_PID_DMA(uint8_t *data, uint16_t len)
{
    float p, i, d;
    char *start_ptr, *end_ptr;

    // 1. 查找帧头 '{' 和帧尾 '@'
    start_ptr = strchr((char *)data, '{');
    end_ptr = strchr((char *)data, '@');

    if (start_ptr && end_ptr && (end_ptr > start_ptr))
    {
        // 2. 将 '@' 替换为结束符，方便 sscanf 解析
        *end_ptr = '\0';

        // 3. 解析中间的三个浮点数 (从 start_ptr + 1 开始)
        if (sscanf(start_ptr + 1, "%f,%f,%f", &p, &i, &d) == 3)
        {
            Tuning->Kp = p;
            Tuning->Ki = i;
            Tuning->Kd = d;
            printf("\r\nPID Updated: P=%.2f, I=%.2f, D=%.2f\r\n", Tuning->Kp, Tuning->Ki, Tuning->Kd);
        }
        else
        {
            printf("\r\nParse Error, Format: {kp,ki,kd@}\r\n");
        }
    }
}

void TUNING_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        Parse_PID_DMA(rx_buffer, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}
