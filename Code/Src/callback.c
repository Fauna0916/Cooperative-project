#include "callback.h"
#include "openmv.h"

extern BNO080_State_t bno_state;
extern uint8_t bno_rx_buffer[BNO_READ_SIZE];

extern uint8_t debug_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim7) // 0.01s
    {
        Control_Update();

        static uint8_t debug_cnt = 0;
        debug_cnt++;
        if(debug_cnt >=10)
        {
            debug_cnt = 0;
            debug_flag = 1;
        }
        
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    OpenMV_RxEventCallback(huart, Size);
    TUNING_RxEventCallback(huart, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    OpenMV_ErrorCallback(huart);
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