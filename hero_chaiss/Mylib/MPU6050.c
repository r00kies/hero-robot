/*********************************************************************************
**                      CPU                        	      MPU6050
**                 -----------------
**             /|\|              XIN|-
**              | |             P7.6|->	--------------------- 3 SCL	23
**              --|             P7.5|-> --------------------- 4 SDA	24
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**********************************************************************************/

#include "r_cg_macrodriver.h"
#include "MPU6050.h"
#include "math.h"
#include "r_cg_userdefine.h"
#include "pid.h"
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#define SDA               P7.5 /*P7_bit.no5*/
#define SDA_INPUT         PM7.5 /*PM7_bit.no5*/


#define SCL               P7.6 /*P7_bit.no6*/
#define SCL_INPUT         PM7.6 /*PM7_bit.no6*/

unsigned char ACK;
#define MPU6050_TRUE  1
#define MPU6050_FALSE 0
#define IIDLY 5
unsigned char mpu6050_buffer[14];
S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET,MAG_OFFSET;			
unsigned char	GYRO_OFFSET_OK = 1;
unsigned char	ACC_OFFSET_OK = 1;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST,HMC5883L_MAG_LAST;		

//////////////////////////////////////////////////////////////////////////
#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.0152672				
#define Gyro_Gr	0.0002663				
#define FILTER_NUM 19
//#define FILTER_NUM 1

//float Kp =1.0f;   
 //float Ki =0.0002f;
#define Kp 2.5f 
#define Ki 0.008f 
#define halfT 0.015f                  
// half the sample period
//#define halfT 0.005f

float LastMag_AVG_X=0;
float LastMag_AVG_Y=0;
float LastMag_AVG_Z=0;

S_FLOAT_XYZ Q_angle;

S_INT16_XYZ ACC_AVG,MAG_AVG;			
S_FLOAT_XYZ GYRO_I;				
S_FLOAT_XYZ EXP_ANGLE;		
S_FLOAT_XYZ DIF_ANGLE;		
S_FLOAT_XYZ Q_ANGLE;			

int	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	
int normAcc;

#define TIMEprepare 0.05
float Accbuffer_x[FILTER_NUM]={0};					   
float Accbuffer_y[FILTER_NUM]={0};
float Accbuffer_z[FILTER_NUM]={0};

#define  IIR_ORDER     4      //ê1ó?IIR??2¨?÷μ??×êy
float b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //?μêyb
float a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//?μêya
float InPut_IIR[3][IIR_ORDER+1] = {0};
float OutPut_IIR[3][IIR_ORDER+1] = {0};

double q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
double exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

/* End user code. Do not edit comment generated here */

/* Start user code for adding. Do not edit comment generated here */
void CreateIICPort(void)
{
	SDA_INPUT=0;    //set output
	SCL_INPUT=0;    //set output
	SDA=1;
	SCL=1;
	PM4.3=0;
    

}
void DelayUs(unsigned short US)
{
    unsigned char i;
    while(US--)
    {
        NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
    }
}

void delayms( int ms)
{
int  i,j;
for(i=0;i<=ms;i++)
	for(j=0;j<=3200;j++);
	//NOP();
	//dataconvert();
}
/*******************************************************************
                     Start iic bus
********************************************************************/
void Start_I2c(void)
{
  SDA=1;   /* send start data signal */
  SCL=1;
  DelayUs(IIDLY);    /* wait more than 4.7us*/
  SDA=0;         /* send start signal */
  DelayUs(IIDLY);    /* wait more than 4.7us*/
  SCL=0; 
}
/*******************************************************************
                      Stop IIC Bus
********************************************************************/
void Stop_I2c(void)
{
  SDA=0;
  SCL=1;
  DelayUs(IIDLY);    /* wait more than 4.7us*/
  SDA=1;
  DelayUs(IIDLY);    /* wait more than 4.7us*/
}




/*******************************************************************
 				Send one byte data or address
********************************************************************/
void  SendByte(unsigned char c)
{
    unsigned char BitCnt;
    SCL=0;
    for(BitCnt=0;BitCnt<8;BitCnt++)
    {
        if((c<<BitCnt)&0x80)SDA=1;
        else  SDA=0;
        DelayUs(IIDLY);    /* wait more than 4.7us*/
        SCL=1;
        DelayUs(IIDLY);    /* wait more than 4.7us*/
        SCL=0;
        NOP();
        NOP();
    }
    SDA_INPUT=1;
    DelayUs(IIDLY);    /* wait more than 4.7us*/
    SCL=1;
    DelayUs(IIDLY);    /* wait more than 4.7us*/
    if(SDA==1)ACK=0;
    else ACK=1;
    SCL=0;
    SDA_INPUT=0;
}






/*******************************************************************/
//				Receive one byte data
/********************************************************************/	
unsigned char  RcvByte(void)
{
    unsigned char retc;
    unsigned char BitCnt;
    SDA_INPUT=1;
    retc=0;
    NOP();
    NOP();
    for(BitCnt=0;BitCnt<8;BitCnt++)
    {
        SCL=0;
 	DelayUs(IIDLY);    /* wait more than 4.7us*/
        SCL=1;
        DelayUs(IIDLY);    /* wait more than 4.7us*/
        retc=retc<<1;
        if(SDA==1)retc=retc+1;
  }
  SCL=0;
  SDA_INPUT=0;
  return(retc);
}




/********************************************************************/
/********************************************************************/
void Ack_I2c(unsigned char a)
{
    if(a==0)SDA=0;
    else SDA=1;
    DelayUs(IIDLY);    /* wait more than 4.7us*/
    SCL=1; 
    DelayUs(IIDLY);    /* wait more than 4.7us*/
    SCL=0;
}




/*******************************************************************
for 24C32/64/128/256
********************************************************************/
unsigned char ISendStrB(unsigned char sla,unsigned short suba,unsigned char *s,unsigned char no)
{
    unsigned char i;
    unsigned char temp;
    SDA_INPUT=0;
    SCL_INPUT=0;
    Start_I2c();
    SendByte(sla);
    if(ACK==0)return(0);
    temp=(unsigned char)(suba>>8);
    SendByte(temp);
    if(ACK==0)return(0);
    temp=(unsigned char)suba;
    SendByte(temp);
    if(ACK==0)return(0);
    for(i=0;i<no;i++)
    {   
        SendByte(*s);
        if(ACK==0)return(0);
        s++;
    } 
    Stop_I2c();
    SCL=1;
    SDA=1;
    return(1);
}

/*******************************************************************
for 24C02/04/08/16/
********************************************************************/
unsigned char ISendStrS(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no)
{
    unsigned char i;
    unsigned char temp;
    SDA_INPUT=0;
    SCL_INPUT=0;
    Start_I2c();
    SendByte(sla);
    if(ACK==0)return(0);
    temp=suba;
    SendByte(temp);
    if(ACK==0)return(0);
    for(i=0;i<no;i++)
    {   
        SendByte(*s);
        if(ACK==0)return(0);
        s++;
    } 
    Stop_I2c();
    SCL=1;
    SDA=1;
    return(1);
}





/*******************************************************************
for 24C32/64/128/256
********************************************************************/
unsigned char IRcvStrB(unsigned char  sla,unsigned short suba,unsigned char  *s,unsigned char  no)
{
   unsigned char  i;
   unsigned char  temp;
   SDA_INPUT=0;
   SCL_INPUT=0;
   Start_I2c();
   SendByte(sla);
   if(ACK==0)return(0);
   temp=(unsigned char)(suba>>8);
   SendByte(temp);
   if(ACK==0)return(0);
   temp=(unsigned char)suba;
   SendByte(temp);
   if(ACK==0)return(0);
   Start_I2c();
   SendByte(sla+1);
   if(ACK==0)return(0);
   for(i=0;i<no-1;i++)
   {
      *s=RcvByte();
      Ack_I2c(0);
      s++;
    }
   *s=RcvByte();
   Ack_I2c(1);
  Stop_I2c();
  SCL=1;
  SDA=1;
  return(1);
}

/*******************************************************************
for 24C02/04/08/16/
********************************************************************/
unsigned char IRcvStrS(unsigned char  sla,unsigned char suba,unsigned char  *s,unsigned char  no)
{
   unsigned char  i;
   unsigned char  temp;
   SDA_INPUT=0;
   SCL_INPUT=0;
   Start_I2c();
   SendByte(sla);
   if(ACK==0)return(0);
   temp=suba;
   SendByte(temp);
   if(ACK==0)return(0);
   Start_I2c();
   SendByte(sla+1);
   if(ACK==0)return(0);
   for(i=0;i<no-1;i++)
   {
      *s=RcvByte();
      Ack_I2c(0);
      s++;
    }
   *s=RcvByte();
   Ack_I2c(1);
  Stop_I2c();
  SCL=1;
  SDA=1;
  return(1);
}
/*******************************************************************
for 24C32/64/128/256
********************************************************************/
unsigned char MPU6050_I2C_WriteByte(unsigned char Addr, unsigned char dat)
{
	ISendStrS(DEVICE_ADDRESS,Addr,&dat,1);
	return 1;
}

unsigned char HMC5883L_I2C_WriteByte(unsigned char Addr, unsigned char dat)
{
	ISendStrS(HMC5883L_Addr,Addr,&dat,1);
	return 1;
}
unsigned char HMC5883L_I2C_READByte(unsigned char Addr)
{	unsigned char dat;
	IRcvStrS(HMC5883L_Addr,Addr,&dat,1);
	return dat;
}

int Read_HMC5883L(void)//读取
{
	char BUF1[7];
	
	
	//delay_ms(5);
	if(HMC5883L_I2C_READByte(0x09)&0x01)
	{
	BUF1[1]=HMC5883L_I2C_READByte(HMC5883L_Output_X_MSB);//OUT_X_H
	BUF1[2]=HMC5883L_I2C_READByte(HMC5883L_Output_X_LSB);//OUT_X_L

	BUF1[3]=HMC5883L_I2C_READByte(HMC5883L_Output_Y_MSB);//OUT_Y_L_A
	BUF1[4]=HMC5883L_I2C_READByte(HMC5883L_Output_Y_LSB);//OUT_Y_H_A
	
        BUF1[5]=HMC5883L_I2C_READByte(HMC5883L_Output_Z_MSB);//OUT_Z_L_A
	BUF1[6]=HMC5883L_I2C_READByte(HMC5883L_Output_Z_LSB);//OUT_Z_H_A
	
	HMC5883L_MAG_LAST.x=(BUF1[1] << 8) | BUF1[2]-MAG_OFFSET.x; //Combine MSB and LSB of X Data output register
	HMC5883L_MAG_LAST.y=(BUF1[3] << 8) | BUF1[4]-MAG_OFFSET.y; //Combine MSB and LSB of Z Data output register
  	HMC5883L_MAG_LAST.z=(BUF1[5] << 8) | BUF1[6]-MAG_OFFSET.z; //Combine MSB and LSB of Z Data output register

	//Magn_y=(Magn_y*HMC5883L_GAIN_Y)/10000;
  	if(HMC5883L_MAG_LAST.x>0x7fff) HMC5883L_MAG_LAST.x-=0xffff;	  
  	if(HMC5883L_MAG_LAST.y>0x7fff) HMC5883L_MAG_LAST.y-=0xffff;
  	if(HMC5883L_MAG_LAST.z>0x7fff) HMC5883L_MAG_LAST.z-=0xffff;
	return 1;
	}
	return 0;
}
void Magdataprepare(void)
{
	unsigned char i=0;

	static int Magbuffer_x[5]={0};					   
	static int Magbuffer_y[5]={0};
	static int Magbuffer_z[5]={0};	
	
	if(Read_HMC5883L()==1)
	{
			for(i=1;i<5;i++){
				Magbuffer_x[i-1]=Magbuffer_x[i];
				Magbuffer_y[i-1]=Magbuffer_y[i];
				Magbuffer_z[i-1]=Magbuffer_z[i];
			}
					
			Magbuffer_x[4]=HMC5883L_MAG_LAST.x;
			Magbuffer_y[4]=HMC5883L_MAG_LAST.y;
			Magbuffer_z[4]=HMC5883L_MAG_LAST.z;
		
			for(i=0;i<5;i++){	//取数组内的值进行求和再取平均
		   		MAG_AVG.x+=Magbuffer_x[i];
				MAG_AVG.y+=Magbuffer_y[i];
				MAG_AVG.z+=Magbuffer_z[i];
			}		  
		
			MAG_AVG.x=MAG_AVG.x/5;
			MAG_AVG.y=MAG_AVG.y/5;
			MAG_AVG.z=MAG_AVG.z/5;			 
			MAG_AVG.x=HMC5883L_MAG_LAST.x;
			MAG_AVG.y=HMC5883L_MAG_LAST.y;
			MAG_AVG.z=HMC5883L_MAG_LAST.z;
	}	
	else
	{
			MAG_AVG.x=LastMag_AVG_X;
			MAG_AVG.y=LastMag_AVG_Y;
			MAG_AVG.z=LastMag_AVG_Z;
	}

	LastMag_AVG_X=MAG_AVG.x;
	LastMag_AVG_Y=MAG_AVG.y;
	LastMag_AVG_Z=MAG_AVG.z;
	//Q_angle.z  =atan2((double)MAG_AVG.y,(double)MAG_AVG.x) * (180 / 3.14159265) + 180;
	
		
}
//**************************************

//**************************************
void InitMPU6050(void)
{
	unsigned char tval;
	do 
	{
	
		MPU6050_I2C_WriteByte(PWR_MGMT_1, 0x01);	
		MPU6050_I2C_WriteByte(CONFIG_6050, 0x03);
		MPU6050_I2C_WriteByte(GYRO_CONFIG, 0x18);
		MPU6050_I2C_WriteByte(ACCEL_CONFIG,  0x09);
		//MPU6050_I2C_WriteByte(INT_PIN_CFG, 0x42);   //使能旁路I2C
		//MPU6050_I2C_WriteByte(USER_CTRL, 0x40);     //使能旁路I2C
	
		/*MPU6050_I2C_WriteByte(PWR_MGMT_1, 0x00);	
		MPU6050_I2C_WriteByte(CONFIG_6050, 0x02);
		MPU6050_I2C_WriteByte(GYRO_CONFIG, 0x18);
		MPU6050_I2C_WriteByte(ACCEL_CONFIG, 0x11);*/
		IRcvStrS(DEVICE_ADDRESS,WHO_AM_I,&tval,1);
	} while (tval!=0x68);
	if (tval)
	{
		DelayUs(1);
	}
	

	//HMC5883L_I2C_WriteByte(HMC5883L_ConfigurationRegisterA,0x78);  
	//delayms(10);
	//HMC5883L_I2C_WriteByte(HMC5883L_ConfigurationRegisterB,0x20);  
	//delayms(10);
	//HMC5883L_I2C_WriteByte(HMC5883L_ModeRegister,0x00);
	//delayms(100);
	Q_angle.z=0;
	ACC_OFFSET_OK=0;
	ACC_OFFSET.x=-0;
	ACC_OFFSET.y=-0;
	ACC_OFFSET.z=7168;
	
	GYRO_OFFSET_OK=0;
	GYRO_OFFSET.x=-0;
	GYRO_OFFSET.y=0;
	GYRO_OFFSET.z=0;
	
	MAG_OFFSET.x=0;
	MAG_OFFSET.y=0;
	MAG_OFFSET.z=0;
	
}

/**********************************************************************
//MPU6050_Last
*******************************************************************************/
void MPU6050_Read(void)
{
	IRcvStrS(DEVICE_ADDRESS,ACCEL_XOUT_H,mpu6050_buffer,14);
	MPU6050_ACC_LAST.x=(((( int)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.x;
	MPU6050_ACC_LAST.y=(((( int)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.y;
	MPU6050_ACC_LAST.z=(((( int)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);// - ACC_OFFSET.z;

	//MPU6050_ACC_LAST.x>>=1;
	//MPU6050_ACC_LAST.y>>=1;
	//MPU6050_ACC_LAST.z>>=1;

	
	MPU6050_GYRO_LAST.x=(((( int)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.x;
	MPU6050_GYRO_LAST.y=(((( int)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.y;
	MPU6050_GYRO_LAST.z=(((( int)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.z;
	
}



/**********************************************************************
//
*******************************************************************************/

void MPU6050_Dataanl(void)
{		MPU6050_Read();
		if(!GYRO_OFFSET_OK)
	{
		static  long	tempgx=0,tempgy=0,tempgz=0;
		static unsigned char cnt_g=0;
		if(cnt_g==0)
		{
			GYRO_OFFSET.x=0;
			GYRO_OFFSET.y=0;
			GYRO_OFFSET.z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		tempgx+= MPU6050_GYRO_LAST.x;
		tempgy+= MPU6050_GYRO_LAST.y;
		tempgz+= MPU6050_GYRO_LAST.z;
		if(cnt_g==200)
		{
			GYRO_OFFSET.x=tempgx/cnt_g;
			GYRO_OFFSET.y=tempgy/cnt_g;
			GYRO_OFFSET.z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)
	{
		static  long	tempax=0,tempay=0,tempaz=0;
		static unsigned char cnt_a=0;

		if(cnt_a==0)
		{
			ACC_OFFSET.x = 0;
			ACC_OFFSET.y = 0;
			ACC_OFFSET.z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= MPU6050_ACC_LAST.x;
		tempay+= MPU6050_ACC_LAST.y;
		tempaz+= MPU6050_ACC_LAST.z;
		if(cnt_a==200)
		{
			ACC_OFFSET.x=tempax/cnt_a;
			ACC_OFFSET.y=tempay/cnt_a;
			ACC_OFFSET.z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			return;
		}
		cnt_a++;		
	}

}
/*******************************************************************
for 24C32/64/128/256
********************************************************************/
void Prepare_Data(void)
{
	static unsigned char filter_cnt=0;
	long temp1=0,temp2=0,temp3=0;
	unsigned char i;
/*	float hn[FILTER_NUM]={		//FIR滤波常数（凯赛窗），带通频率100Hz,带阻频率200hz。加速度滤波
	-0.0048,0.0000,0.0155,0.0186,-0.0152,
 	-0.0593,-0.0345,0.1045,0.2881,0.3739,
 	 0.2881,0.1045,-0.0345,-0.0593,
 	 -0.0152,0.0186,0.0155,0.0000,-0.0048
		  };
	 float Acctmp_x=0;
	 float Acctmp_y=0;
	 float Acctmp_z=0;*/

	MPU6050_Read();
	MPU6050_Dataanl();

	/*Accbuffer_x[0] = MPU6050_ACC_LAST.x;
	Accbuffer_y[0] = MPU6050_ACC_LAST.y;
	Accbuffer_z[0]= MPU6050_ACC_LAST.z;


	for(i=0;i<FILTER_NUM;i++)
	{
			Acctmp_x+=hn[i]*Accbuffer_x[i];
			Acctmp_y+=hn[i]*Accbuffer_y[i];
			Acctmp_z+=hn[i]*Accbuffer_z[i];	
	}
	ACC_AVG.x =  Acctmp_x ;
	ACC_AVG.y =  Acctmp_y ;
	ACC_AVG.z =  Acctmp_z ;
			for(i=0;i<FILTER_NUM-1;i++)
		{
			Accbuffer_x[FILTER_NUM-1-i]=Accbuffer_x[FILTER_NUM-2-i];
			Accbuffer_y[FILTER_NUM-1-i]=Accbuffer_y[FILTER_NUM-2-i];
			Accbuffer_z[FILTER_NUM-1-i]=Accbuffer_z[FILTER_NUM-2-i];
		}
	//filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;*/
	
	
	ACC_AVG.x =  IIR_I_Filter(MPU6050_ACC_LAST.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	ACC_AVG.y =  IIR_I_Filter(MPU6050_ACC_LAST.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	ACC_AVG.z =  IIR_I_Filter(MPU6050_ACC_LAST.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	Q_angle.z += MPU6050_GYRO_LAST.z*Gyro_G*0.03;
}
/*******************************************************************
for 24C32/64/128/256
********************************************************************/
void Get_Attitude(void)
{
	double gx,  gy,  gz,  ax,  ay,  az;	
	double norm,vx, vy, vz,ex, ey, ez;	
	double q0q0;
	double q0q1;
	double q0q2;
	double q0q3;
	double q1q1;
	double q1q2 ;
	double q1q3 ;
	double q2q2 ;
	double q2q3 ;
	double q3q3;
	double delta_2=0;
	 q0q0 = q0*q0;
	 q0q1 = q0*q1;
	 q0q2 = q0*q2;
	 q0q3 = q0*q3;
	 q1q1 = q1*q1;
	 q1q2 = q1*q2;
	 q1q3 = q1*q3;
	 q2q2 = q2*q2;
	 q2q3 = q2*q3;
	 q3q3 = q3*q3;

	gx=MPU6050_GYRO_LAST.x*Gyro_Gr;
	gy=MPU6050_GYRO_LAST.y*Gyro_Gr;
	gz=MPU6050_GYRO_LAST.z*Gyro_Gr;
	ax=ACC_AVG.x;
	ay=ACC_AVG.y;
	az=ACC_AVG.z;
	
	if(ax*ay*az==0)
		return;

	norm = sqrtf(ax*ax + ay*ay + az*az); 
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;

	vx = 2*(q1q3 - q0q2);												
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	ex = (ay*vz - az*vy) ;                           		
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	exInt = exInt + ex * Ki;								
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	gx = gx + Kp*ex + exInt;					   							
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   					

	//q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;//?t?×±??¨·¨
	//q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	///q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	//q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	
	delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);//ò??×áú???a?t·¨
  	q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			
  	q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  	q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  	q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;	
	

	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//Q_angle.z = -atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; ;
	Q_angle.y  =- asinf(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
	Q_angle.x = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
	//Q_angle.z  =GYRO_I.z;

}
////////////////////////////////////////////////////////////////////////////////
#if 1
void IMUupdate()
{    
	double gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz;
static double q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   
  double norm;
  double hx, hy, hz, bx, bz;
  double vx, vy, vz, wx, wy, wz;
  double ex, ey, ez;
   float delta_2=0;
  // ???????????
  double q0q0 = q0*q0;
  double q0q1 = q0*q1;
  double q0q2 = q0*q2;
  double q0q3 = q0*q3;
  double q1q1 = q1*q1;
  double q1q2 = q1*q2;
  double q1q3 = q1*q3;
  double q2q2 = q2*q2;
  double q2q3 = q2*q3;
  double q3q3 = q3*q3;
    const static double FACTOR = 0.002;
  	gx=MPU6050_GYRO_LAST.x*Gyro_Gr;
	gy=MPU6050_GYRO_LAST.y*Gyro_Gr;
	gz=MPU6050_GYRO_LAST.z*Gyro_Gr;
	ax=ACC_AVG.x;
	ay=ACC_AVG.y;
	az=ACC_AVG.z;
	
	mx=MAG_AVG.x;
	my=MAG_AVG.y;
	mz=MAG_AVG.z;
	if(ax*ay*az==0)
 	      return;
 norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  norm = sqrt(mx*mx + my*my + mz*mz);
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;
 
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;
	 
  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2*q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
 
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

 if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
 {
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }			   							//???gz????????????????,??????????????

  // integrate quaternion rate and normalise						   //????????
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
 // Q_angle.z = -atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; ;
  Q_angle.y  = asinf(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
  Q_angle.x = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
  Q_angle.z=gz*4;

}
#endif
/* End user code. Do not edit comment generated here */
