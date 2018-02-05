#ifndef __TASK_DOOR_H__
#define __TASK_DOOR_H__
#include <stm32f4xx.h>
#include "pwm.h"
#include "usart2.h"
#include "task_led.h"

#define PWM_DOOR  TIM12->CCR2//舵机控制舱门
#define PWM_ELSE  TIM12->CCR1//预留备用
void door_task(void *pdata);
void DOOR_control();
#endif