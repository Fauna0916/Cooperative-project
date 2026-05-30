#ifndef __HCSR04_H
#define __HCSR04_H

#include "tim.h"
#include <stdbool.h>

#define HCSR04_TIMER &htim6

#define TRIG_PORT HCSR04_TRIG_GPIO_Port
#define TRIG_PIN HCSR04_TRIG_Pin
#define ECHO_PORT HCSR04_ECHO_GPIO_Port
#define ECHO_PIN HCSR04_ECHO_Pin


void HCSR04_Init(void);
void HCSR04_TriggerUpdate(void);
void HCSR04_EXTI_Callback(uint16_t GPIO_Pin);

float HCSR04_GetDistance(void);
bool HCSR04_IsObstacleDetected(void);

#endif
