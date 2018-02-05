#include "main.h"
#include "includes.h"
#include "task_door.h"
//��ʼ����
#define START_TASK_PRIO      	  10
#define START_STK_SIZE  				64
OS_STK START_TASK_STK[START_STK_SIZE];
//can����
#define CAN_TASK_PRIO       		5
#define CAN_STK_SIZE  		    		256
OS_STK CAN_TASK_STK[CAN_STK_SIZE];
//��������
#define CONTROL_TASK_PRIO       			6
#define CONTROL_STK_SIZE  		    		256
OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
//��������
#define DBUG_TASK_PRIO       	   9
#define DBUG_STK_SIZE  		    		256
OS_STK DBUG_TASK_STK[DBUG_STK_SIZE];
//���ſ�������
#define DOOR_TASK_PRIO       		  7
#define DOOR_STK_SIZE  					  64
OS_STK DOOR_TASK_STK[DOOR_STK_SIZE];
//��������
#define LED_TASK_PRIO       			8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
#define LED_STK_SIZE  		    		64
OS_STK LED_TASK_STK[LED_STK_SIZE];
int main(void)
{

	  
    int i = 0;
    BSP_Init();  
    delayms(500);//delay 500ms�� �ȴ�mpu6050�ϵ��ȶ�     
    while(MPU6050_Initialization() == 0xff) 
    {
        i++;     //���һ�γ�ʼ��û�гɹ����Ǿ�����һ��                     
        if(i>10) //�����ʼ��һֱ���ɹ����Ǿ�ûϣ���ˣ�������ѭ����������һֱ��
        {
            while(1) 
            {
                LED_RED_TOGGLE();
                delayms(500);
                
            }
        }  
    }
		    pid_init_struct();
        MPU6050_Interrupt_Configuration();
		    TIM6_Start();
        Data_Read();
        iwdg_init();
        //V_CTIM_Driver_Mode(ENTER_SPEED_CURRENT_MODE,5000 ,300);
        delayms(500);  //����ģʽѡ��ָ���Ҫ�ȴ�����������ģʽ������������ʱҲ������ȥ����	
        delay_init(168);
		    OSInit();  	 				//��ʼ��UCOSII
  	    OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	      OSStart();
		    
 


}
  //��ʼ����
  void start_task(void *pdata)
   {  
		    OS_CPU_SR cpu_sr=0;
		    pdata = pdata;
        OSStatInit();							 
				OS_ENTER_CRITICAL();			   
				OSTaskCreate(can_task,(void *)0,(OS_STK*)&CAN_TASK_STK[CAN_STK_SIZE-1],CAN_TASK_PRIO);						   
				OSTaskCreate(control_task,(void *)0,(OS_STK*)&CONTROL_TASK_STK[CONTROL_STK_SIZE-1],CONTROL_TASK_PRIO);	 				   
				OSTaskCreate(dbug_task,(void *)0,(OS_STK*)&DBUG_TASK_STK[DBUG_STK_SIZE-1],DBUG_TASK_PRIO);	 				   
				OSTaskCreate(door_task,(void *)0,(OS_STK*)&DOOR_TASK_STK[DOOR_STK_SIZE-1],DOOR_TASK_PRIO);	 				   
				OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);	 				   	 				   
				OSTaskSuspend(START_TASK_PRIO);	
				OS_EXIT_CRITICAL();				
   }
