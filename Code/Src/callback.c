#include "callback.h"
#include "openmv.h"

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
    BNO_EXTI_Callback(GPIO_Pin);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BNO_I2C_MasterRxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BNO_I2C_ErrorCallback(hi2c);
}