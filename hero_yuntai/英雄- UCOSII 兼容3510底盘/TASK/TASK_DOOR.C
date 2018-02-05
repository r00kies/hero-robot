#include "task_door.h"




//舱门控制任务
void door_task(void *pdata)
{
	while(1)
	{
		DOOR_control();
    Turn_Off_LEDs();
		delay_ms(10);
	}									 
}
//舱门控制
void DOOR_control()
{
	       if(RC_Ctl.rc.s2==1)
    {
        if((RC_Ctl.rc.ch2-1024)>500)//关闭
        {
					 PWM_DOOR=3;//舵机DOOR闭
					 LED_RED_ON(); 
         GPIO_ResetBits(GPIOC,GPIO_Pin_0);//关闭红外信号
        }
        else if((RC_Ctl.rc.ch2-1024)<-500)//打开
        {
					 PWM_DOOR=8;		//舵机DOOR开		 
					 LED_RED_OFF();
        GPIO_SetBits(GPIOC,GPIO_Pin_0);//发射红外信号
        }
	 }

 }