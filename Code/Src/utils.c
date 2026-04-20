#include "utils.h"

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
    printf("letf:%d,%d\r\n", Encoder_GetLeftData()->total_ticks, Encoder_GetLeftData()->speed_rpm);
    printf("right:%d,%d\r\n", Encoder_GetRightData()->total_ticks, Encoder_GetRightData()->speed_rpm);
    printf("x:%.1f,y:%.1f,theta:%.1f,v:%.1f,w:%.1f\r\n", Odometry_GetState()->x, Odometry_GetState()->y, Odometry_GetState()->theta, Odometry_GetState()->linear_vel, Odometry_GetState()->angular_vel);

    HAL_Delay(100);
}