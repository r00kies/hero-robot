#ifndef __PWM_H__
#define __PWM_H__
#include <stm32f4xx.h>
void PWM1_Configuration(void);
void PWM2_Configuration(void);
void pwm_out(u16 PWM_CH3,u16 PWM_CH4);
void Rammer_moto_run(void);
void TIM12_CH12_Init(u16 arr,u16 psc);

#define PWM1  TIM3->CCR3
#define PWM2  TIM3->CCR4

#define PWM_DOOR  TIM12->CCR2//������Ʋ���
#define PWM_ELSE  TIM12->CCR1//Ԥ������

#define PWM3  TIM4->CCR3
#define PWM4  TIM4->CCR4
#endif


