/*********************************************************************************
**                      CPU                        	      MPU6050
**                 -----------------
**             /|\|              XIN|-
**              | |             P7.6|->	--------------------- 3 SCL	
**              --|             P7.5|-> --------------------- 4 SDA
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**                |                 |
**********************************************************************************/

#ifndef MPU6050_H
#define MPU6050_H

#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
extern unsigned char mpuBuf[17];
/* Start user code for function. Do not edit comment generated here */
#define DEVICE_ADDRESS	0xD0

#define MPU6050_READ	0x01
#define MPU6050_WRITE	0x00

#define SDA_BIT	BIT1
#define SCL_BIT	BIT0

#define SDA_DIR	P2DIR
#define SCL_DIR	P2DIR

#define SDA_IN	P2IN
#define SDA_OUT	P2OUT
#define SCL_OUT	P2OUT

#define SDA_IN_MODE		SDA_DIR &= ~SDA_BIT
#define SDA_OUT_MODE	SDA_DIR |= SDA_BIT
#define SCL_OUT_MODE	SCL_DIR |= SCL_BIT

#define I2C_CONFIG	{SDA_OUT_MODE;SCL_OUT_MODE;}

#define SDA_H	SDA_OUT |= SDA_BIT	
#define SDA_L	SDA_OUT &= ~SDA_BIT

#define SCL_H	SCL_OUT |= SCL_BIT	
#define SCL_L	SCL_OUT &= ~SCL_BIT

#define READ_SDA_IN	(SDA_IN & SDA_BIT)==SDA_BIT
void delayms( int ms);
//****************************************
// MPU6050 address
//****************************************
#define	SMPLRT_DIV		0x19	//
#define	CONFIG_6050			0x1A	//fittler
#define	GYRO_CONFIG		0x1B	//
#define	ACCEL_CONFIG	0x1C	//
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//
#define	WHO_AM_I			0x75	
#define	SlaveAddress	0xD0	
#define INT_PIN_CFG     0x37    
#define USER_CTRL       0x6A    


#define ABS(x) ((x)>=0?(x):(-(x)))

#define	HMC5883L_Addr   0x3C	  
#define HMC5883L_ConfigurationRegisterA  0x00
#define HMC5883L_ConfigurationRegisterB  0x01
#define HMC5883L_ModeRegister            0x02
#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 		 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 		 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 		 0x08
#define HMC5883L_StatusRegister		 0x09
#define HMC5883L_ID_A										 0x0A
#define HMC5883L_ID_B 									 0x0B
#define HMC5883L_ID_C 									 0x0C

#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)
int Read_HMC5883L(void);
void Magdataprepare();
void IMUupdate();
void delay( int ms);

typedef struct{
	int x;
	int y;
	int z;
}S_INT16_XYZ;
extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST,HMC5883L_MAG_LAST;		
extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET,MAG_OFFSET,MAG_AVG,GYRO_AVG;			
extern unsigned char	GYRO_OFFSET_OK;
extern unsigned char	ACC_OFFSET_OK;

typedef struct{
	float x;
	float y;
	float z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_angle;		


void CreateIICPort(void);
void DelayUs(unsigned short US);
void Start_I2c(void);
void Stop_I2c(void);
void  SendByte(unsigned char c);
unsigned char  RcvByte(void);
void Ack_I2c(unsigned char a);
unsigned char ISendStrB(unsigned char sla,unsigned short suba,unsigned char *s,unsigned char no);    //for 24C32/64/128/256
unsigned char ISendStrS(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no);     //for 24C02/04/08/16/
unsigned char IRcvStrB(unsigned char  sla,unsigned short suba,unsigned char  *s,unsigned char  no);  //for 24C32/64/128/256
unsigned char IRcvStrS(unsigned char  sla,unsigned char suba,unsigned char  *s,unsigned char  no);  //for 24C02/04/08/16/
void InitMPU6050(void);
void MPU6050_Dataanl(void);
void MPU6050_Read(void);

void Prepare_Data(void);
void Get_Attitude(void);

/* End user code. Do not edit comment generated here */
#endif
