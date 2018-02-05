#ifndef _TASK_CONTROL_H_
#define _TASK_CONTROL_H_
#include "stm32f4xx.h"
#include "math.h"
#include "MPU6050.h"
#include "mpu6050_driver.h"
#include "mpu6050_process.h"
#include "usart1.h"
#include "TASK_DBUG.h"
#include "usart2.h"
#include "task_can.h"
#include "PWM.h"  
#include "math.h"
#define PWM_MOTOR_MIN    2000	      
#define PWM_MOTOR_MAX    4000		
#define abs(x) ((x)>0? (x):(-(x)))

typedef struct {
  float expectation;          
  float Err_k;			  
  float Err_k_1;		  
  float Err_k_2;		 
  float SumErr;             
  float Kp;				 
  float Ti;				
  float Td;				 
  float Ouput_deltaUk;		
  float Ouput_deltaUk_Max;		
  float Ouput_deltaUk_Min;		
  float PID_Integral_Max;				
  float PID_Integral_Min;				
} PID_Struct;
extern PID_Struct PID_Rammer_moto;
extern PID_Struct PID_Yaw_V_Struct;	
extern PID_Struct PID_Roll_V_Struct;			    
extern PID_Struct PID_Pitch_V_Struct;
extern PID_Struct PID_Yaw_P_Struct;				
extern PID_Struct PID_Roll_P_Struct;			   
extern PID_Struct PID_Pitch_P_Struct;
extern PID_Struct Chassis_Position_Control;
extern s32 expect_position[5];
void PID_Rammer_moto_Init(PID_Struct *PID);
void PID_Init(PID_Struct *PID);
void pid_init_struct(void);
float PIDz_Calculate(PID_Struct* PID, float measured, float expect);
float PID_Calculate(PID_Struct* PID, float measured, float expect);
s16 transe_M_angle(s16 angle,u16 first_angle);
s16 pwm_confine(s16 Pwm);
void Position_control();
extern s16 pwm_set_1,pwm_set_2,pwm_set_3,pwm_set_4;
void Friction_wheel_control (s16 speed,u8 mode);
void moto_ctrl();
void Gimban_ctrl(void);
void Gimban_Camera_ctrl(void);
void control_task(void *pdata); 
u8 keyboard_data(u16 key);
void keyboard(void);
u8 keyboard_jump_1(u8 key);
u8 keyboard_jump_2(u8 key);
#endif