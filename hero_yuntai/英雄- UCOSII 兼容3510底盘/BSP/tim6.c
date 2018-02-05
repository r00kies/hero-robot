#include "tim6.h"
#include "mpu6050_process.h"
#include "mpu6050_driver.h"
#include "HMC5883L.h"
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 2000;
    TIM_TimeBaseInit(TIM6,&tim);
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

void TIM6_DAC_IRQHandler(void)  
{
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{

//		MPU6050_Angle_Calculate(MPU6050_Real_Data.Gyro_X* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Y* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Z* (3.14159265/180.0),
//        MPU6050_Real_Data.Accel_X,
//        MPU6050_Real_Data.Accel_Y,
//        MPU6050_Real_Data.Accel_Z);
//        IMU_AHRSupdate(MPU6050_Real_Data.Gyro_X* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Y* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Z* (3.14159265/180.0),
//        MPU6050_Real_Data.Accel_X,
//        MPU6050_Real_Data.Accel_Y,
//        MPU6050_Real_Data.Accel_Z,
//        mxyz[0],
//        mxyz[1],
//        mxyz[2]);
        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);  
  }
}