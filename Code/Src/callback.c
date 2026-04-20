#include "callback.h"


extern BNO080_State_t bno_state;
extern uint8_t bno_rx_buffer[BNO_READ_SIZE];

static uint8_t huart2_flag = False;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim7) // 0.01
    {
        Encoder_Update();
        Odometry_Update();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        huart2_flag = True;
    }
}

void UART2_handle(void)
{
    static uint8_t rx_buf[20];
    if (huart2_flag)
    {
        huart2_flag = False;
        HAL_UART_Receive_IT(&huart2, rx_buf, 5);
        HAL_UART_Transmit_IT(&huart2, rx_buf, 5);
    }
    return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BNO_INT_Pin)
    {
        if (bno_state == BNO080_IDLE)
        {
            bno_state = BNO080_READING;
            if (HAL_I2C_Master_Receive_DMA(&BNO080_I2C, BNO080_I2C_ADDR, bno_rx_buffer, BNO_READ_SIZE) != HAL_OK)
            {
                bno_state = BNO080_ERROR;
            }
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == BNO080_I2C.Instance)
    {
        if (bno_state == BNO080_READING)
        {
            bno_state = BNO080_DATA_READY;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == BNO080_I2C.Instance)
    {
        bno_state = BNO080_ERROR;
    }
}