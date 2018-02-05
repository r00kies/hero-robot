#include "bsp.h"

void BSP_Init(void){
    
    /* Configure the NVIC Preemption Priority Bits */
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   
        LED_Configuration();            //ָʾ�Ƴ�ʼ��
        Key_Configuration(); 
        Buzzer_Configuration();
	      RELAY_init();                   //��ʼ���̵���
        LASER_Configuration();
        CAN1_Configuration();            //��ʼ��CAN
        CAN2_Configuration(); 
        USART1_Configuration();          //����1��ʼ��
        USART2_Configuration();
        USART3_Configuration();
        TIM6_Configuration();
        Encoder_Configuration();
        PWM1_Configuration();  
        PWM2_Configuration(); 
	      TIM12_CH12_Init(99,16799);       //500hz 20ms����
       //�趨ռ�ձ�Ϊ1000����ʼ��Ħ���ֵ��
        PWM1 =1000;
        PWM2 =1000;
}


