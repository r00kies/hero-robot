#include "stmflash.h"

/* Base address of the Flash sectors */ 
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define PAGE_BYTES	2048
#define PAGE_NUM		64

static u8 clear_flash(u16 sector);
static u8 get_act_page(void);
static u8 write_float(u32 start_addr, u16 cnt, float data);
static u8 write_int(u32 start_addr, u16 cnt, u32 data);
static float read_float(u32 start_addr, u16 cnt);
static u32 read_int(u32 start_addr, u16 cnt);
extern u32 first_yaw_angle,first_pitch_angle; 
extern s16 Friction_wheel_set_speed;
extern s16 Set_Rammer_Moto_Speed;
extern int correct;
extern s16 limit_pitch_down,limit_pitch_up,limit_yaw_left,limit_yaw_right;//机械限位值
u8 Data_Save(void)
{	
	u8 act_page_num = 255;	//可以使用的页码
	u32 act_page_start = 0;	//该页起始地址
	
	act_page_num = get_act_page();
	if(act_page_num>(PAGE_NUM-2))
	{
		if(clear_flash(FLASH_Sector_11))
			act_page_num = 0;
		else
			return 0;	//擦除失败
	}
	
	act_page_start = FLASH_USER_START_ADDR + (act_page_num * PAGE_BYTES);
	FLASH_Unlock();
	
	if(FLASH_ProgramWord(act_page_start, 0x12345678) == FLASH_COMPLETE);
  else
		return 0;	//写入失败
	
 
	u16 data_cnt = 1;
	if(!write_float(act_page_start, data_cnt++, Chassis_Position_Control.Kp) == FLASH_COMPLETE) return 0;	//写入失败
	if(!write_float(act_page_start, data_cnt++, Chassis_Position_Control.Ti) == FLASH_COMPLETE) return 0;	//写入失败
	if(!write_float(act_page_start, data_cnt++, Chassis_Position_Control.Td) == FLASH_COMPLETE) return 0;	//写入失败
  if(!write_int(act_page_start, data_cnt++,expect_position[1]) == FLASH_COMPLETE) return 0;	//写入失败
	if(!write_int(act_page_start, data_cnt++,expect_position[2]) == FLASH_COMPLETE) return 0;	//写入失败
	if(!write_int(act_page_start, data_cnt++,expect_position[3]) == FLASH_COMPLETE) return 0;	//写入失败
	if(!write_int(act_page_start, data_cnt++,expect_position[4]) == FLASH_COMPLETE) return 0;	//写入失败
	FLASH_Lock(); 

	return 1;
}
u8 Data_Read(void)
{
	u8 act_page_num = 255;	//可以使用的页码
	u32 act_page_start = 0;	//该页起始地址
	
	act_page_num = get_act_page();
	if(act_page_num==0)
		return 0;
	else
		act_page_num -= 1;
	act_page_start = FLASH_USER_START_ADDR + (act_page_num * PAGE_BYTES);
	
	u16 data_cnt = 1;
	Chassis_Position_Control.Kp = read_float(act_page_start, data_cnt++);
	Chassis_Position_Control.Ti = read_float(act_page_start, data_cnt++);
	Chassis_Position_Control.Td = read_float(act_page_start, data_cnt++);
	expect_position[1] = read_int(act_page_start, data_cnt++);
	expect_position[2] = read_int(act_page_start, data_cnt++);
	expect_position[3] = read_int(act_page_start, data_cnt++);
	expect_position[4] = read_int(act_page_start, data_cnt++);  
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
static u8 clear_flash(u16 sector)
{
	u8 _return = 0;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if (FLASH_EraseSector(sector, VoltageRange_3) != FLASH_COMPLETE)
    { 
      _return = 0;//失败
    }
	FLASH_Lock(); 
	_return = 1;	//成功
	return _return;
}
static u8 get_act_page(void)
{
	for(u8 i=0;i<PAGE_NUM;i++)
	{
		if((*(__IO uint32_t*)(FLASH_USER_START_ADDR+i*PAGE_BYTES))==0xffffffff)
		{
			return i;
		}
	}
	return 0xff;
}
static u8 write_float(u32 start_addr, u16 cnt, float data)
{
	if(cnt>510)
		return 0;

	u32 temp;
	temp = *(uint32_t *)(&data);
	if(FLASH_ProgramWord(start_addr+(cnt*4), temp) == FLASH_COMPLETE)
		return 1;
	else 
		return 0;	//写入失败
}
static u8 write_int(u32 start_addr, u16 cnt, u32 data)
{
	if(cnt>510)
		return 0;

	if(FLASH_ProgramWord(start_addr+(cnt*4), data) == FLASH_COMPLETE)
		return 1;
	else 
		return 0;	//写入失败
}
static float read_float(u32 start_addr, u16 cnt)
{
	u32 temp = *(__IO uint32_t*)(start_addr+(cnt*4));
	return *(float *)(&temp);
}
static u32 read_int(u32 start_addr, u16 cnt)
{
	return *(__IO uint32_t*)(start_addr+(cnt*4));
}
