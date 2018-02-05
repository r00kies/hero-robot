#ifndef __PWM_H__
#define __PWM_H__

void PWM_Configuration(void);

void Moto_Init(void);
void P_I_D_figure_up(void);
void SetMotorVoltage(float f_tVal);
void MotorSpeedOut(float f_Val); 
#endif
