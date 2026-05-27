#include "hcsr04.h"

static volatile uint16_t echo_start_time = 0;
static volatile float current_distance_cm = 999.0f; 
static uint32_t last_trigger_tick = 0;

/**
 * @brief  微秒级延时函数 (利用配置为 1us 的 TIM6)
 * @param  us: 延时微秒数
 */
static void delay_us(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(HCSR04_TIMER);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(HCSR04_TIMER) - start) < us)
        ;
}

/**
 * @brief  初始化超声波模块
 */
void HCSR04_Init(void)
{
    // 启动时间戳定时器
    HAL_TIM_Base_Start(HCSR04_TIMER);

    // 拉低 TRIG 引脚，准备就绪
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    current_distance_cm = 999.0f;
}

/**
 * @brief  触发一次超声波测量。需定期调用（已内置防止频繁触发的逻辑）
 */
void HCSR04_TriggerUpdate(void)
{
    // 超声波声波在空气中衰减需要时间，建议触发间隔 > 60ms
    if (HAL_GetTick() - last_trigger_tick < 60)
    {
        return;
    }
    last_trigger_tick = HAL_GetTick();

    // 产生 10us 以上的高电平脉冲触发超声波
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(12); // 阻塞 12us 对 400MHz 系统完全无影响
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}


void HCSR04_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ECHO_PIN)
    {
        if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
        {
            // 上升沿：记录开始时间
            echo_start_time = __HAL_TIM_GET_COUNTER(HCSR04_TIMER);
        }
        else
        {
            // 下降沿：计算时间差
            uint16_t echo_end_time = __HAL_TIM_GET_COUNTER(HCSR04_TIMER);
            uint16_t echo_duration = echo_end_time - echo_start_time;

            // 距离(cm) = 时间(us) / 58.0
            // 加入合法性过滤 (比如超声波最大测距 400cm，约 23200us)
            if (echo_duration > 100 && echo_duration < 25000)
            {
                current_distance_cm = echo_duration / 58.0f;
            }
            else
            {
                current_distance_cm = 999.0f; // 无效数据或超出范围
            }
        }
    }
}


float HCSR04_GetDistance(void)
{
    return current_distance_cm;
}


bool HCSR04_IsObstacleDetected(void)
{
    return (current_distance_cm <= OBSTACLE_THRESHOLD_CM);
}