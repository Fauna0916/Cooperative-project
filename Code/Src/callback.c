#include "callback.h"
#include "hcsr04.h"
#include "robot_task.h"
#include "lora_tx.h"

extern uint8_t debug_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim7) // 0.001s
    {
        Control_Update();
        static uint8_t debug_cnt = 0;
        debug_cnt++;
        if(debug_cnt >=100)
        {
            debug_cnt = 0;
            debug_flag = 1;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    TUNING_RxEventCallback(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    LoRa_UART_TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BNO_EXTI_Callback(GPIO_Pin);
    HCSR04_EXTI_Callback(GPIO_Pin);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BNO_I2C_MasterRxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BNO_I2C_ErrorCallback(hi2c);
}