#include "control.h"
#include "math.h"
#include "MPU6050.h"
#include "Uart.h"
#include "main.h"
float Time_dt;
float pid_roll;
float pid_pitch;
float pid_yaw;
float Motor_Ail=0.0;					 
float Motor_Ele=0.0;					
float Motor_Thr=0.0;					 
float Motor_Rud=0.0;					 

PID_Struct PID_Power_Struct;	
PID_Struct PID_Position_Struct;			    
PID_Struct PID_Pitch_V_Struct;			
PID_Struct PID_Yaw_P_Struct;				
PID_Struct PID_Roll_P_Struct;			   
PID_Struct PID_Pitch_P_Struct;	
PID_Struct PID_Moto_Drive;
Moto_Struct  Moto_Drive;
/*************************
      ��ʼ��PID����
*************************/
void PID_V_Init(PID_Struct *PID)
{
  PID->expectation         = 0.0;            //ң�ظ�������ֵ
  PID->Err_k			   = 0.0;            //��ǰ���ֵe(k)
  PID->Err_k_1		       = 0.0;           //k-1ʱ�����ֵe(k-1)
  PID->Err_k_2		       = 0.0;           //k-2ʱ�����ֵe(k-2)
  PID->SumErr              = 0.0;			//����
  PID->Kp				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ti				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Td				   = 0.0;           //΢��ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID�����������U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = POWER_Period;		    //������������ֵ
  PID->Ouput_deltaUk_Min   = -POWER_Period;		    //�����������Сֵ
  PID->PID_Integral_Max    = 20.0;		     //���ƻ��������ֵ
  PID->PID_Integral_Min    = -20.0;			//���ƻ�������Сֵ
}
void PID_P_Init(PID_Struct *PID)
{
  PID->expectation         = 0.0;            //ң�ظ�������ֵ
  PID->Err_k			   = 0.0;            //��ǰ���ֵe(k)
  PID->Err_k_1		       = 0.0;           //k-1ʱ�����ֵe(k-1)
  PID->Err_k_2		       = 0.0;           //k-2ʱ�����ֵe(k-2)
  PID->SumErr              = 0.0;			//����
  PID->Kp				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ti				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Td				   = 0.0;           //΢��ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID�����������U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = PWM_Period;		    //������������ֵ
  PID->Ouput_deltaUk_Min   = -PWM_Period;		    //�����������Сֵ
  PID->PID_Integral_Max    = 7000.0;		     //���ƻ��������ֵ
  PID->PID_Integral_Min    = -7000.0;			//���ƻ�������Сֵ
}
void PID_Position_Init(PID_Struct *PID)
{
  PID->expectation         = 0.0;            //ң�ظ�������ֵ
  PID->Err_k			   = 0.0;            //��ǰ���ֵe(k)
  PID->Err_k_1		       = 0.0;           //k-1ʱ�����ֵe(k-1)
  PID->Err_k_2		       = 0.0;           //k-2ʱ�����ֵe(k-2)
  PID->SumErr              = 0.0;			//����
  PID->Kp				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ti				   = 0.0;           //����ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Td				   = 0.0;           //΢��ϵ����ͨ���������ߵ�PID������д��Flash
  PID->Ouput_deltaUk       = 0.0;		    //PID�����������U(k) - U(k-1)
  PID->Ouput_deltaUk_Max   = SPEED_LIMIT;		    //������������ֵ
  PID->Ouput_deltaUk_Min   = -SPEED_LIMIT;		    //�����������Сֵ
  PID->PID_Integral_Max    = 500.0;		     //���ƻ��������ֵ
  PID->PID_Integral_Min    = -500.0;			//���ƻ�������Сֵ
}

void pid_init_struct(void)
{
	PID_P_Init(&PID_Power_Struct);
    PID_V_Init(&PID_Moto_Drive);
    PID_Position_Init(&PID_Position_Struct);
//	PID_Init(&PID_Roll_V_Struct);
//	PID_Init(&PID_Pitch_V_Struct);
//	
//	PID_Init(&PID_Yaw_P_Struct);
//	PID_Init(&PID_Roll_P_Struct);
//	PID_Init(&PID_Pitch_P_Struct);
	
	
}

/***************************************************************
PID����-����ʽ
***************************************************************/
float PIDz_Calculate(PID_Struct* PID, float measured, float expect)
{
     float K_p = PID->Kp;
     float T_d = PID->Td;
     float T_i = PID->Ti;
     float increment;
    PID->Err_k=PID->Err_k_1;
    PID->Err_k_1=PID->Err_k_2;
    PID->Err_k_2=(expect-measured)*100;
    increment = (K_p+T_i+T_d)*PID->Err_k_2-(K_p+2*T_d)*PID->Err_k_1+T_d*PID->Err_k;
    PID->Ouput_deltaUk+=increment;
    if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
    {
        PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;
    }
    if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
    {
        PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;
    }
    
  return PID->Ouput_deltaUk;
}

/***************************************************************
PID����-λ��ʽ
***************************************************************/
float PID_Calculate(PID_Struct* PID, float measured, float expect)
{
  float Value_Proportion;  
  float Value_Integral;
  float Value_Derivative;	

  PID->expectation =  expect;
  PID->Err_k = (PID->expectation - measured);
  PID->SumErr+= PID->Err_k;

  //P I D
  Value_Proportion    = PID->Kp * PID->Err_k;
  Value_Integral      =  PID->SumErr * PID->Ti;	
  Value_Derivative  = PID->Kp * PID->Td * (PID->Err_k - PID->Err_k_1);

  if(Value_Integral > PID->PID_Integral_Max)
  {
    PID->SumErr -= PID->Err_k;
	Value_Integral = PID->PID_Integral_Max;
  }
  if(Value_Integral < PID->PID_Integral_Min)
  {
  	PID->SumErr -= PID->Err_k;
    Value_Integral = PID->PID_Integral_Min;
  }
  
  PID->Ouput_deltaUk = Value_Proportion + Value_Integral + Value_Derivative;

  if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;}
  if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;}

  PID->Err_k_1 = PID->Err_k;	  //����k-1�����ֵ
  
  return PID->Ouput_deltaUk;
}

 
