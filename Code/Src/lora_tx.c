#include "lora_tx.h"
#include "ds3231.h"
#include <stdio.h>
#include <string.h>

static uint8_t lora_tx_buffer[256];
static bool is_lora_busy = false;
static uint32_t last_trigger_tick = 0;

/* --- Arch detection edge-trigger state --- */
static bool arch_last_level = false; /* previous debounced level */
static bool arch_armed = true;       /* ready to fire on next rising edge */

void LoRa_Init(void)
{
    is_lora_busy = false;
    arch_last_level = false;
    arch_armed = true;
    // DS3231_SetTime(0, 0, 8, 10, 39, 30);
}

/**
 * @brief  Arch detection — send exactly one LoRa message per arch passage.
 * @param  arch_detected: true when HCSR04 sees the arch overhead
 * @param  start_tick:    mission start tick (for race duration)
 *
 * Rising-edge triggered; re-arms only after the signal falls back to false.
 */
void LoRa_ProcessArchTrigger(bool arch_detected, uint32_t start_tick)
{
    /* --- Rising edge: arch just entered --- */
    if (arch_detected && !arch_last_level && arch_armed)
    {
        arch_armed = false; /* one shot per passage */

        /* Debounce / rate-limit: don't fire if last TX was < 150 ms ago */
        if (HAL_GetTick() - last_trigger_tick < 150)
        {
            return;
        }
        last_trigger_tick = HAL_GetTick();

        /* Busy check */
        if (HAL_UART_GetState(LORA_UART) == HAL_UART_STATE_BUSY_TX || is_lora_busy)
        {
            arch_armed = true; /* re-arm to try again next cycle */
            return;
        }

        /* --- Build payload with DS3231 RTC time --- */
        DS3231_Time rtc;
        char time_str[16] = "00:00:00";

        if (DS3231_GetTime(&rtc) == HAL_OK)
        {
            sprintf(time_str, "%02u:%02u:%02u",
                    (unsigned int)rtc.hours,
                    (unsigned int)rtc.minutes,
                    (unsigned int)rtc.seconds);
        }

        /* Race duration from start_tick */
        uint32_t current_tick = HAL_GetTick();
        uint32_t duration_s = (current_tick - start_tick) / 1000;
        uint16_t min = (uint16_t)(duration_s / 60);
        uint16_t sec = (uint16_t)(duration_s % 60);

        char payload[128];
        int payload_len = sprintf(payload,
                                  "Gp:%s,Name:%s,Time:%s,Dur:%02d:%02d\r\n",
                                  TEAM_NUMBER, TEAM_NAME, time_str, min, sec);

        /* Assemble frame: [AddrH, AddrL, Channel, payload...] */
        lora_tx_buffer[0] = LORA_TARGET_ADDR_H;
        lora_tx_buffer[1] = LORA_TARGET_ADDR_L;
        lora_tx_buffer[2] = LORA_CHANNEL;
        memcpy(&lora_tx_buffer[3], payload, (size_t)payload_len);

        is_lora_busy = true;
        if (HAL_UART_Transmit_IT(LORA_UART, lora_tx_buffer,
                                 (uint16_t)(payload_len + 3)) != HAL_OK)
        {
            is_lora_busy = false;
        }
    }
    /* --- Falling edge: re-arm for next passage --- */
    else if (!arch_detected && arch_last_level)
    {
        arch_armed = true;
    }

    arch_last_level = arch_detected;
}

/**
 * @brief  Legacy task-data sender (kept for backward compatibility).
 */
void LoRa_SendTaskData_NonBlocking(uint32_t start_tick)
{
    if (HAL_GetTick() - last_trigger_tick < 150)
    {
        return;
    }
    last_trigger_tick = HAL_GetTick();

    if (HAL_UART_GetState(LORA_UART) == HAL_UART_STATE_BUSY_TX || is_lora_busy)
    {
        return;
    }

    char payload[128];
    uint32_t current_tick = HAL_GetTick();
    uint32_t duration_s = (current_tick - start_tick) / 1000;
    uint16_t min = (uint16_t)(duration_s / 60);
    uint16_t sec = (uint16_t)(duration_s % 60);
    uint32_t sys_s = current_tick / 1000;

    int payload_len = sprintf(payload, "Gp:%s,Name:%s,Time:%02u:%02u:%02u,Dur:%02d:%02d\r\n",
                              TEAM_NUMBER, TEAM_NAME,
                              (unsigned int)((sys_s / 3600) % 24),
                              (unsigned int)((sys_s / 60) % 60),
                              (unsigned int)(sys_s % 60),
                              min, sec);

    lora_tx_buffer[0] = LORA_TARGET_ADDR_H;
    lora_tx_buffer[1] = LORA_TARGET_ADDR_L;
    lora_tx_buffer[2] = LORA_CHANNEL;
    memcpy(&lora_tx_buffer[3], payload, (size_t)payload_len);

    is_lora_busy = true;
    if (HAL_UART_Transmit_IT(LORA_UART, lora_tx_buffer, (uint16_t)(payload_len + 3)) != HAL_OK)
    {
        is_lora_busy = false;
    }
}

void LoRa_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LORA_UART->Instance)
    {
        is_lora_busy = false;
    }
}