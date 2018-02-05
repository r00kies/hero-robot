#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>

#include "encoder.h"
#include "led.h"
#include "pwm.h"
#include "tim2.h"
#include "delay.h"
#include "uart.h"
#include "can.h"
#include "ad.h"
#include "flash.h"
#include "key.h"
#include "iwdg.h"
#include "DataScope_DP.h"
#include "DEBUG.h"
#include "control.h"
#include "BSP.H"
#define abs(x) ((x)>0? (x):(-(x)))
//#define set_speed_max  650
#define PWM_Period     5000
#define POWER_Period   20
#define SPEED_LIMIT   200
extern float g_f_measured_speed;
#endif
