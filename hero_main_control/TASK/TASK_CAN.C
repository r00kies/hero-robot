#include "task_can.h"
static s16 Value_HH,Value_HL,Value_LH,Value_LL;
static s16 Value_H,Value_L,Value;
static unsigned char can_tx_success_flag = 0;
s16 pwm_set_1,pwm_set_2,pwm_set_3,pwm_set_4;
extern u8 Gear;
s32 measured_Position[6];
s16 mearsured_speed[6];
u8 can_id[6];
//can通信任务
void can_task(void *pdata)
{
	while(1)
	{
		//ALL_can_tx();
		delay_ms(10);
	}
}

void ALL_can_tx()
{
      if(RC_Ctl.rc.s2==2||RC_Ctl.rc.s2==3)
			 {
			            LED_RED_ON();
				       if(RC_Ctl.rc.s1==1)
				          Position_control();
							 else if(RC_Ctl.rc.s1==2)
							 {
                   moto_ctrl();
				         
                 V_CTIM_Driver_Speed(CAN1,
                 (short)(pwm_confine(pwm_set_1))/2/(Gear+1),
                 (short)(pwm_confine(pwm_set_2))/2/(Gear+1),
                 (short)(pwm_confine(pwm_set_3))/2/(Gear+1),
                 (short)(pwm_confine(pwm_set_4))/2/(Gear+1));
							 }
			 }
			 else
			 {

              V_CTIM_Driver_Speed(CAN1,0,0,0,0);
			        LED_RED_OFF();
			 } 

}

/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}
/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg canRxMsg;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
      
         CAN_Receive(CAN1, CAN_FIFO0, &canRxMsg);
         switch(canRxMsg.StdId)
         {
             case 1:
							      Value_HH=canRxMsg.Data[0];
                    Value_HL=canRxMsg.Data[1];
                    Value_LH=canRxMsg.Data[2];
                    Value_LL=canRxMsg.Data[3];
                    measured_Position[1]=(Value_HH<<24)|(Value_HL<<16)|(Value_LH<<8)|(Value_LL);
						        Value_H=canRxMsg.Data[4];
                    Value_L=canRxMsg.Data[5];
						        mearsured_speed[1]=Value_H<<8|Value_L;
						        can_id[1]=canRxMsg.Data[6];
						        break;
             
             case 2:
							      Value_HH=canRxMsg.Data[0];
                    Value_HL=canRxMsg.Data[1];
                    Value_LH=canRxMsg.Data[2];
                    Value_LL=canRxMsg.Data[3];
                    measured_Position[2]=(Value_HH<<24)|(Value_HL<<16)|(Value_LH<<8)|(Value_LL);
						        Value_H=canRxMsg.Data[4];
                    Value_L=canRxMsg.Data[5];
						        mearsured_speed[2]=Value_H<<8|Value_L;
						        can_id[2]=canRxMsg.Data[6];
						        break;
             case 3:
							      Value_HH=canRxMsg.Data[0];
                    Value_HL=canRxMsg.Data[1];
                    Value_LH=canRxMsg.Data[2];
                    Value_LL=canRxMsg.Data[3];
                    measured_Position[3]=(Value_HH<<24)|(Value_HL<<16)|(Value_LH<<8)|(Value_LL);
						        Value_H=canRxMsg.Data[4];
                    Value_L=canRxMsg.Data[5];
						        mearsured_speed[3]=Value_H<<8|Value_L;
						        can_id[3]=canRxMsg.Data[6];
						        break;
             case 4:
							      Value_HH=canRxMsg.Data[0];
                    Value_HL=canRxMsg.Data[1];
                    Value_LH=canRxMsg.Data[2];
                    Value_LL=canRxMsg.Data[3];
                    measured_Position[4]=(Value_HH<<24)|(Value_HL<<16)|(Value_LH<<8)|(Value_LL);
						        Value_H=canRxMsg.Data[4];
                    Value_L=canRxMsg.Data[5];
						        mearsured_speed[4]=Value_H<<8|Value_L;
						        can_id[4]=canRxMsg.Data[6];
						        break;
             default:;
         }
    }
}

/*************************************************************************
                      RoboModule_Driver_Reset
函数描述：让挂接在CAN总线上的某个驱动器复位
传入参数：CAN_ID
*************************************************************************/
void RoboModule_Driver_Reset(unsigned char CAN_ID)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Mode_Chioce
函数描述：让挂接在CAN总线上的某个驱动器进入某种模式
传入参数：CAN_ID
传入参数：ENTER_X_MODE
*************************************************************************/
void RoboModule_Driver_Mode_Chioce(unsigned char CAN_ID,unsigned char ENTER_X_MODE)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = ENTER_X_MODE;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_PWM_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在PWM模式下赋值
传入参数：CAN_ID
传入参数：PWM_Value
*************************************************************************/
void RoboModule_Driver_PWM_Mode_Set(unsigned char CAN_ID,short PWM_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((PWM_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(PWM_Value&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

/*************************************************************************
                      RoboModule_Driver_Current_Mode_Set
函数描述：给挂接在CAN总线上的某个驱动器在Current模式下赋值
传入参数：CAN_ID
传入参数：Current_Value
*************************************************************************/
void RoboModule_Driver_Current_Mode_Set(unsigned char CAN_ID,short Current_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID

    if(Current_Value > 2000)
    {
        Current_Value = 2000;
    }
    else if(Current_Value < -2000)
    {
        Current_Value = -2000;
    }

    tx_message.Data[0] = (unsigned char)((Current_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Current_Value&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

void RoboModule_Driver_Location_Mode_Set(unsigned char CAN_ID,short PWM_Value,short Speed_Value,int Location_Value)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID

    if(PWM_Value > 5000)
    {
        PWM_Value = 5000;
    }
    else if(PWM_Value < -5000)
    {
        PWM_Value = -5000;
    }

    if(Speed_Value > 1000)
    {
        Speed_Value = 1000;
    }
    else if(Speed_Value < -1000)
    {
        Speed_Value = -1000;
    }
    
    if(Location_Value > 5000000)
    {
        Location_Value = 5000000;
    }
    else if(Location_Value < -5000000)
    {
        Location_Value = -5000000;
    }

    tx_message.Data[0] = (unsigned char)((PWM_Value>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(PWM_Value&0xff);
    tx_message.Data[2] = (unsigned char)((Speed_Value>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Speed_Value&0xff);
    tx_message.Data[4] = (unsigned char)((Location_Value>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Location_Value>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Location_Value>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Location_Value&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}
/*************************************************************************
                      RoboModule_Driver_Mode
函数描述：让挂接在CAN总线上的某个驱动器进入某种模式
传入参数：CAN_ID
传入参数：ENTER_X_MODE
传入参数：PWM_Value
传入参数：limit_power
*************************************************************************/
void V_CTIM_Driver_Mode(unsigned char ENTER_X_MODE,short PWM_Value,short limit_power)
{
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    tx_message.StdId = 0x300;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = ENTER_X_MODE;
    tx_message.Data[1] = (unsigned char)((PWM_Value>>8)&0xff);
    tx_message.Data[2] = (unsigned char)(PWM_Value&0xff);
    tx_message.Data[3] = (unsigned char)((limit_power>>8)&0xff);
    tx_message.Data[4] = (unsigned char)(limit_power&0xff);
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}
/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void V_CTIM_Driver_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}
