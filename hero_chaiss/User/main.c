#include "main.h"
extern uint8_t Moto_pid_1000Hz;
extern uint8_t Attitude_500Hz;
extern uint8_t Motor_100Hz;
extern uint8_t Com_10Hz;
extern uint8_t Set_2Hz;
extern uint8_t SystemInitReady ;
void main(void)
{

	  	if ( SysTick_Config(SystemCoreClock/1000))
  { 
    while (1);
  }
	pid_init_struct();
	FLASH_Unlock();
	EE_INIT();
	EE_READ_PID();
	LED_Configuration();
	Encoder_Configuration();
	Encoder_Start();
	LED_Configuration();
	Moto_Init();
	TIM4_Init();
    CAN_Configuration();
//	iwdg_init();
	ADC_Configuration();
	USART1_Configuration();
	CAN_Configuration();
	SystemInitReady=1;
	delay_ms(1000);
    while(1)
    {
	 if(Moto_pid_1000Hz)
	{
	       Moto_pid_1000Hz=0;			 //清除标志位
		     send_position(CAN1,Moto_Drive.Measured_Position,Moto_Drive.Measured_Speed,Moto_Drive.Can_id);
         ANO_DT_Data_Exchange();
	}
		if(Attitude_500Hz)			
	{
	     Attitude_500Hz=0;	
		 
	}
    	if(Motor_100Hz)				
	{
		Motor_100Hz=0;
	}
		if(Com_10Hz)
		{
            calibrate_IMU();
			Com_10Hz=0;
			{
				LED_GREEN_ON();
			}
		}
      if(Set_2Hz)
			{
			static u8 i;
			i++;
     
			if(i%2)
			{
		  LED_RED_ON();
			}
      else
			{
      LED_RED_OFF();
			}
       Set_2Hz=0;
		 }			 
      
    }

}
