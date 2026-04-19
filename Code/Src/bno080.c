#include "bno080.h"
#include <math.h>



#define QUATERNION_SCALE 0.00006103515625f

ALIGN_32BYTES(uint8_t bno_rx_buffer[BNO_READ_SIZE]) __attribute__((section(".ARM.__at_0x24000000")));
volatile BNO080_State_t bno_state = BNO080_IDLE;
static BNO080_Data_t bno_data = {0};


static void BNO080_EnableGameRotationVector(void)
{
    uint8_t set_feature_cmd[21] = {
        21, 0, 2, 0,
        0xFD, 0x08, 0, 0, 0,
        0x10, 0x27, 0, 0, // 10000 us (100Hz)
        0, 0, 0, 0,
        0, 0, 0, 0};
    HAL_I2C_Master_Transmit(&BNO080_I2C, BNO080_I2C_ADDR, set_feature_cmd, 21, 100);
    HAL_Delay(50);
}

void BNO080_HardwareReset(void)
{
    HAL_Delay(50);
    HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

    // wait for the sensor to boot up
    uint32_t wait_tick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET)
    {
        if (HAL_GetTick() - wait_tick > 500)
            break;
    }

    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET)
    {
        HAL_I2C_Master_Receive(&BNO080_I2C, BNO080_I2C_ADDR, bno_rx_buffer, BNO_READ_SIZE, 100);
    }
    HAL_Delay(50);
}

void BNO080_Init()
{
    BNO080_HardwareReset();
    BNO080_EnableGameRotationVector();
    bno_data.last_update_tick = HAL_GetTick();
    bno_state = BNO080_IDLE;
}


BNO080_State_t BNO080_Update(void)
{
    if (bno_state == BNO080_IDLE)
    {
        if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin))
        {
            bno_state = BNO080_READING;
            if (HAL_I2C_Master_Receive_DMA(&BNO080_I2C, BNO080_I2C_ADDR, bno_rx_buffer, BNO_READ_SIZE) != HAL_OK)
            {
                bno_state = BNO080_ERROR;
            }
        }
    }

    if (bno_state == BNO080_DATA_READY)
    {
        uint8_t channel = bno_rx_buffer[2];

        if (channel == 3 && bno_rx_buffer[9] == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
        {
            int16_t q_i = (bno_rx_buffer[14] << 8) | bno_rx_buffer[13];
            int16_t q_j = (bno_rx_buffer[16] << 8) | bno_rx_buffer[15];
            int16_t q_k = (bno_rx_buffer[18] << 8) | bno_rx_buffer[17];
            int16_t q_real = (bno_rx_buffer[20] << 8) | bno_rx_buffer[19];

            bno_data.i = q_i * QUATERNION_SCALE;
            bno_data.j = q_j * QUATERNION_SCALE;
            bno_data.k = q_k * QUATERNION_SCALE;
            bno_data.real = q_real * QUATERNION_SCALE;
            bno_data.quat_accuracy = bno_rx_buffer[11] & 0x03;

            float sqi = bno_data.i * bno_data.i;
            float sqj = bno_data.j * bno_data.j;
            float sqk = bno_data.k * bno_data.k;
            float sqr = bno_data.real * bno_data.real;

            bno_data.yaw = atan2f(2.0f * (bno_data.i * bno_data.j + bno_data.real * bno_data.k), (sqr + sqi - sqj - sqk));

            float sinp = 2.0f * (bno_data.real * bno_data.j - bno_data.k * bno_data.i);
            bno_data.pitch = (fabs(sinp) >= 1.0f) ? copysignf(3.14159265f / 2.0f, sinp) : asinf(sinp);

            bno_data.roll = atan2f(2.0f * (bno_data.real * bno_data.i + bno_data.j * bno_data.k), (sqr - sqi - sqj + sqk));
        }

        bno_data.last_update_tick = HAL_GetTick();
        bno_state = BNO080_READY_TO_READ;          
        return BNO080_READY_TO_READ;
    }

    // overtime watchdog: 300ms
    if (bno_state == BNO080_ERROR || (HAL_GetTick() - bno_data.last_update_tick > 300))
    {
        HAL_I2C_DeInit(&BNO080_I2C);
        HAL_I2C_Init(&BNO080_I2C);
        BNO080_Init();
        return BNO080_ERROR;
    }

    return bno_state;
}

BNO080_Data_t *BNO080_GetLatestData(void)
{
    return &bno_data;
}

void BNO080_DataReaded(void)
{
    bno_state = BNO080_IDLE;
}

