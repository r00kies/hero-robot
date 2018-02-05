#include "main.h"
void TIM4_Init()
{
	TIM_TimeBaseInitTypeDef  TIM_InitStructure;
  NVIC_InitTypeDef	       NVIC_InitStructure;
  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_DeInit(TIM4);
	TIM_InitStructure.TIM_Prescaler =72-1;//72分频
	TIM_InitStructure.TIM_Period =100-1;     //重复计数器值
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV4;
	TIM_InitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM4,&TIM_InitStructure);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//更新中断
		
	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static u8 k,i;
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
	{
		static float out_pwm,AD_out,AD_out_new,AD_out_old,Speed_out;
		k++;
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		Moto_Drive.Measured_Power=AD_Power()*Moto_Drive.current_dir;
		if(k>=10)
		{
			k=0; 
			AD_out_old=AD_out_new;
			Moto_Drive.Measured_Speed=Encoder_Get_CNT()*Moto_Drive.Encoder_Dir;
			Moto_Drive.Measured_Position+=Moto_Drive.Measured_Speed;
			
			if(Can_Moto_Date_struct.MOTO_MODE==ENTER_SPEED_CURRENT_MODE)    
			AD_out_new=PID_Calculate(&PID_Moto_Drive,Moto_Drive.Measured_Speed,Moto_Drive.Speed_Value);
			else if(Can_Moto_Date_struct.MOTO_MODE==ENTER_POSITION_SPEED_CURRENT_MODE)
			{
				i++;
				if(i>=10)
				{
					i=0;
					Speed_out=PID_Calculate(&PID_Position_Struct,Moto_Drive.Measured_Position,Moto_Drive.Expectation_Position);    
				}
				AD_out_new=PID_Calculate(&PID_Moto_Drive,Moto_Drive.Measured_Speed,Speed_out); 
			}
		}
		AD_out=(AD_out_new-AD_out_old)*(1+k)/10+AD_out_old;
		out_pwm=PID_Calculate(&PID_Power_Struct,Moto_Drive.Measured_Power,AD_out*Moto_Drive.Moto_Dir);
		MotorSpeedOut(out_pwm);
	} 
}

