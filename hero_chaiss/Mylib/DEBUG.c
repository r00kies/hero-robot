
//                         Resen                          NFR2401
//                 -----------------
//             /|\|              XIN|-
//              | |             P146|->	--------------------- 3 CS					
//              --|RST          P147|-> --------------------- 4 CSN					
//                |                 |
//                |             P1.4|-> Data Out (UCA1SIMO)   6 MOSI				
//                |                 |
//	          |             P1.3|<- Data In (UCA1SOMI)    7 MISO				
//                |                 |
//                |             P1.5|-> Serial Clock Out (UCA1CLK) 5CLK				
//===========================RF24L01 Port==========================================
#include "main.h"
#include "Uart.h"
#include "DEBUG.h"
#include "MPU6050.h"
#include "control.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
char fly_ready=0;
dt_flag_t f;				
u8 data_to_send[50];	
u16 Rc_Pwm_In[9];
mpu mpu6050;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST,HMC5883L_MAG_LAST;		
S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET,MAG_OFFSET,MAG_AVG,GYRO_AVG;
S_FLOAT_XYZ Q_angle;	
 CanRxMsg rx_message; 
void calibrate_IMU()
{
    if(mpu6050.Gyro_CALIBRATE)
    {
        TIM_Cmd(TIM4, ENABLE);
        Can_Moto_Date_struct.MOTO_MODE = ENTER_SPEED_CURRENT_MODE;
        mpu6050.Gyro_CALIBRATE=0;
    }
        if(mpu6050.Acc_CALIBRATE)
    {
        TIM_Cmd(TIM4, ENABLE);
        Can_Moto_Date_struct.MOTO_MODE=ENTER_POSITION_SPEED_CURRENT_MODE;
        mpu6050.Acc_CALIBRATE=0;
    }

    
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
//	static u8 power_cnt   =	50;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
//	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
//	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;
//
//	if((cnt % power_cnt) == (power_cnt-1))
//		f.send_power = 1;		
	
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(Moto_Drive.Measured_Speed,Moto_Drive.Measured_Power,Q_angle.z,0,0,fly_ready);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(MPU6050_ACC_LAST.x,MPU6050_ACC_LAST.y,MPU6050_ACC_LAST.z,
		MPU6050_GYRO_LAST.x,MPU6050_GYRO_LAST.y,MPU6050_GYRO_LAST.z,
		0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		Rc_Pwm_In[0]=rx_message.Data[0]<<8|rx_message.Data[1];
		Rc_Pwm_In[1]=rx_message.Data[2]<<8|rx_message.Data[3];
		Rc_Pwm_In[2]=rx_message.Data[4]<<8|rx_message.Data[5];

		Rc_Pwm_In[4]=rx_message.Data[6]<<8|rx_message.Data[7];
		Rc_Pwm_In[5]=0;
		Rc_Pwm_In[6]=0;
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(Rc_Pwm_In[0],Rc_Pwm_In[1],Rc_Pwm_In[2],Rc_Pwm_In[3],Rc_Pwm_In[4],Rc_Pwm_In[5],Rc_Pwm_In[6],Rc_Pwm_In[7],0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(Moto_Drive.Measured_Position,Moto_Drive.Measured_Speed,Moto_Drive.Can_id,0,0,0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,PID_Moto_Drive.Kp,PID_Moto_Drive.Ti,PID_Moto_Drive.Td,
				  Moto_Drive.Can_id,Moto_Drive.encoder_dir/1000,Moto_Drive.moto_dir/1000,
				  PID_Power_Struct.Kp,PID_Power_Struct.Ti,PID_Power_Struct.Td);
        
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,PID_Position_Struct.Kp,PID_Position_Struct.Ti,PID_Position_Struct.Td,
                          Moto_Drive.Speed_Value/100,Moto_Drive.limit_power/100,Moto_Drive.Expectation_Position/1000,
				          PID_Yaw_P_Struct.Kp,PID_Yaw_P_Struct.Ti,PID_Yaw_P_Struct.Td);
	}
/////////////////////////////////////////////////////////////////////////////////////
//	else if(f.send_pid3)
//	{
//		f.send_pid3 = 0;
//		ANO_DT_Send_PID(3,PID_Moto_Drive.Kp,PID_Moto_Drive.Ti,0,
//		0,0,0,
//		0,0,0);
//	}

}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{

//	R_UART1_Send(data_to_send, length);
	USART1_Send_Date(USART1,dataToSend , length );

}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static volatile u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i=0;
	for(;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			mpu6050.Acc_CALIBRATE = 1;
		if(*(data_buf+4)==0X02)
			mpu6050.Gyro_CALIBRATE = 1;
		if(*(data_buf+4)==0X03)
		{
			mpu6050.Acc_CALIBRATE = 1;		
			mpu6050.Gyro_CALIBRATE = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{

		}
		if(*(data_buf+4)==0XA0)		
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		
		{
			//Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								
    {
	u16 _temp1,_temp2,_temp3;    
	_temp1=*(data_buf+4);
	_temp2=*(data_buf+5);
        PID_Moto_Drive.Kp  = 0.001*( (_temp1<<8)|_temp2 );
		_temp1=*(data_buf+6);
	_temp2=*(data_buf+7);
        PID_Moto_Drive.Ti  = 0.001*( (_temp1<<8)|_temp2 );
		_temp1=*(data_buf+8);
	_temp2=*(data_buf+9);
         PID_Moto_Drive.Td  = 0.001*( (_temp1<<8)|_temp2 );
		_temp1=*(data_buf+10);
  	_temp2=*(data_buf+11);
        Moto_Drive.Can_id = ( (_temp1<<8)|_temp2 );

		_temp1=*(data_buf+12);
	  _temp2=*(data_buf+13);
        _temp3 = ((_temp1<<8)|_temp2 );
			Moto_Drive.encoder_dir=_temp3;
			if(Moto_Drive.encoder_dir)
				Moto_Drive.Encoder_Dir=1;
			else
			  Moto_Drive.Encoder_Dir=-1;
		_temp1=*(data_buf+14);
	  _temp2=*(data_buf+15);

			_temp3 = ((_temp1<<8)|_temp2 );
			Moto_Drive.moto_dir=_temp3;
      if(Moto_Drive.moto_dir)
				Moto_Drive.Moto_Dir=1.0;
			else
			  Moto_Drive.Moto_Dir=-1.0;

		_temp1=*(data_buf+16);
	_temp2=*(data_buf+17);
        PID_Power_Struct.Kp 	= 0.001*( (_temp1<<8)|_temp2 );
           
		_temp1=*(data_buf+18);
	_temp2=*(data_buf+19);
        PID_Power_Struct.Ti 	= 0.001*( (_temp1<<8)|_temp2 );
		_temp1=*(data_buf+20);
	_temp2=*(data_buf+21);
        PID_Power_Struct.Td 	= 0.001*( (_temp1<<8)|_temp2 );
        ANO_DT_Send_Check(*(data_buf+2),sum);		
				EE_SAVE_PID();
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
	u16 _temp1,_temp2; 

	_temp1=*(data_buf+4);
	_temp2=*(data_buf+5);
        PID_Position_Struct.Kp = 0.001*( (_temp1<<8)|_temp2 );
	_temp1=*(data_buf+6);
	_temp2=*(data_buf+7);
        PID_Position_Struct.Ti = 0.001*( (_temp1<<8)|_temp2 );
    _temp1=*(data_buf+8);
    _temp2=*(data_buf+9);
        PID_Position_Struct.Td =0.001*( (_temp1<<8)|_temp2 );
        
	_temp1=*(data_buf+10);
	_temp2=*(data_buf+11);
    Moto_Drive.Speed_Value  = 0.1*( (_temp1<<8)|_temp2 );
    Moto_Drive.limit_Speed=Moto_Drive.Speed_Value;
    PID_Position_Struct.Ouput_deltaUk_Max= Moto_Drive.limit_Speed;
    PID_Position_Struct.Ouput_deltaUk_Min=-Moto_Drive.limit_Speed;
	_temp1=*(data_buf+12);
	_temp2=*(data_buf+13);
        Moto_Drive.limit_power  = 0.1*( (_temp1<<8)|_temp2 );
    PID_Moto_Drive.Ouput_deltaUk_Max= Moto_Drive.limit_power;
	PID_Moto_Drive.Ouput_deltaUk_Min=-Moto_Drive.limit_power;
    _temp1=*(data_buf+14);
    _temp2=*(data_buf+15);
        Moto_Drive.Expectation_Position = ( (_temp1<<8)|_temp2 );
        
	_temp1=*(data_buf+16);
	_temp2=*(data_buf+17);
        PID_Yaw_P_Struct.Kp	= ( (_temp1<<8)|_temp2 );
	_temp1=*(data_buf+18);
	_temp2=*(data_buf+19);
        PID_Yaw_P_Struct.Ti 	= 0.001*( (_temp1<<8)|_temp2 );
	_temp1=*(data_buf+20);
	_temp2=*(data_buf+21);
        PID_Yaw_P_Struct.Td 	= 0.001*( (_temp1<<8)|_temp2 );
        ANO_DT_Send_Check(*(data_buf+2),sum);
				
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	

        ANO_DT_Send_Check(*(data_buf+2),sum);
				
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	s32 _temp2;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp2= alt;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 i=0;
	u8 sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000/1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
