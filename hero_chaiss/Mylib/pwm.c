#include "main.h"
#define MAX_CONTROL  5000
#define MOTOR_OUT_DEAD_VAL 100
#define MotorOut_max 5000


/*************************************************************************
                              PWM初始化       
*************************************************************************/
void PWM_GPIO_Config()
{
/**************PA0????**************/
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//?????????
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//?????????
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

//	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}
void PWM_Configuration()
{
	/**********??????************/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitSturcture;
  /*************??????**************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//??????
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/***************?TIM???????*****************/
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_Period =PWM_Period;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/***********?TIM?OC??***************/
	TIM_OCInitSturcture.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitSturcture.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitSturcture.TIM_Pulse =1;
  TIM_OCInitSturcture.TIM_OCPolarity = TIM_OCPolarity_High;

//	TIM_OC2Init(TIM2, &TIM_OCInitSturcture);
	TIM_OC3Init(TIM2, &TIM_OCInitSturcture);
	TIM_OC4Init(TIM2, &TIM_OCInitSturcture);
	/***********??************/
	TIM_CtrlPWMOutputs(TIM2,ENABLE);          
	TIM_Cmd(TIM2, ENABLE);	
}

/******************************************************************************
 * ????: Moto_Init
 * ????: ??????GPIOA.0.1.2.3
 * ????: ?
 ******************************************************************************/
void Moto_Init(void)
{
  PWM_GPIO_Config();
  PWM_Configuration();
}


/******************************************************************************
 * 函数名称: SetMotorVoltage
 * 函数功能: 输入信号解析
 * 入口参数: float f_tVal
 ******************************************************************************/
//void SetMotorVoltage(float f_tVal)
//{
//	
//	if(f_tVal>0)
//	{		
//	TIM2->CCR2=f_tVal;
//	GPIO_SetBits(GPIOA,GPIO_Pin_3);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
//	}
//	else
//	{
//	TIM2->CCR2=-f_tVal;
//	GPIO_SetBits(GPIOA,GPIO_Pin_2);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_3);

//	}

//}
void SetMotorVoltage(float f_tVal)
{

	if(f_tVal>0)
	{
    Moto_Drive.current_dir=1;
	TIM2->CCR3=f_tVal;
    TIM2->CCR4=0;
	}
	else
	{
    Moto_Drive.current_dir=-1;
	TIM2->CCR4=-f_tVal;
	TIM2->CCR3=0;
	}

}
/******************************************************************************
 * 函数名称: MotorSpeedOut
 * 函数功能: 电机死区补偿并输出信号
 * 入口参数: void
 ******************************************************************************/
void MotorSpeedOut(float f_Val) 
{

 if(f_Val > 0) 
 f_Val += MOTOR_OUT_DEAD_VAL;
 else if(f_Val < 0) 
 f_Val -= MOTOR_OUT_DEAD_VAL;
 if(f_Val>PWM_Period)
 {
 f_Val=PWM_Period;
 }
 else if(f_Val<-PWM_Period)
 {
 f_Val=-PWM_Period;
 }
 SetMotorVoltage(f_Val);
}


