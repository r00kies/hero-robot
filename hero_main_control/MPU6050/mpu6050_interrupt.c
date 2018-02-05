#include "main.h"
int correct;
void MPU6050_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);   
 
	  gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,GPIO_PinSource5); 
    
    exti.EXTI_Line = EXTI_Line5;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}
void hmc5883_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,  ENABLE);   
 
	  gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,GPIO_PinSource2); 
    
    exti.EXTI_Line = EXTI_Line2;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
	  nvic.NVIC_IRQChannel = EXTI2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

}
//MPU6050 �ⲿ�жϴ�����
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        
        //��ȡMPU6050����,Ϊ��ʹ��̨�Ŀ��Ƹ�ƽ����
        //ʹ��MPU6050�������������Ϊ�ٶȻ�����
        //����ʹ�õ���巵�ػ�е�Ƕ�ֵ���ٶȻ���������������������
        MPU6050_ReadData();
			 MPU6050_Angle.Yaw-=MPU6050_Real_Data.Gyro_Y/180/3.14;
//					MPU6050_Angle_Calculate(MPU6050_Real_Data.Gyro_X* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Y* (3.14159265/180.0),
//        MPU6050_Real_Data.Gyro_Z* (3.14159265/180.0),
//        MPU6050_Real_Data.Accel_X,
//        MPU6050_Real_Data.Accel_Y,
//        MPU6050_Real_Data.Accel_Z);
        //MPU6050_Angle.Yaw-=MPU6050_Real_Data.Gyro_Z/180/3.14;			
        //MPU6050_Data_Filter();
        EXTI_ClearFlag(EXTI_Line5);          
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
//		 if(EXTI_GetITStatus(EXTI_Line8) == SET)
//    {
//        
////        MPU6050_ReadData();                                              
//        EXTI_ClearFlag(EXTI_Line8);          
//        EXTI_ClearITPendingBit(EXTI_Line8);
//    }
}
void EXTI2_IRQHandler()
{
			 if(EXTI_GetITStatus(EXTI_Line2) == SET)
    {
         //HMC58X3_mgetValues(mxyz);
//        mxyz[0]=mxyz[0]*cos(MPU6050_Angle.Pitch/57.2168)+
//        mxyz[1]*sin(MPU6050_Angle.Pitch/57.2168)*sin(MPU6050_Angle.Rool/57.2168)
//        -mxyz[2]*sin(MPU6050_Angle.Pitch/57.2168)*cos(MPU6050_Angle.Rool/57.2168);
//        mxyz[1]=mxyz[1]*cos(MPU6050_Angle.Rool/57.2168)+mxyz[2]*sin(MPU6050_Angle.Rool/57.2168);
        EXTI_ClearFlag(EXTI_Line2);          
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
