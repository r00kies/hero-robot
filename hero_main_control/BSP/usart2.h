#ifndef __USART2_H__
#define __USART2_H__

#include <stm32f4xx.h>
#define KEY_B   0x8000
#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001

typedef struct { 
    struct     {       
		uint16_t ch0;       
		uint16_t ch1;         
		uint16_t ch2;        
		uint16_t ch3;         
		uint8_t  s1;       
		uint8_t  s2;     
		}rc;  
    struct    
 {        
 int16_t x;     
 int16_t y;       
 int16_t z;        
 uint8_t press_l;  
 uint8_t press_r;   
 }mouse;  
    struct  
			{   
uint16_t v;   
}key; 
}RC_Ctl_t; 
extern u8 key_a,key_b,key_d,key_s,key_w,key_z,key_x,key_c,key_v,key_g,key_f,key_r,key_e,key_q,key_ctrl,key_shift;
extern u8 key_jump_a,key_jump_d,key_jump_s,key_jump_w,key_jump_z,key_jump_x,key_jump_c,key_jump_v,key_jump_g,
key_jump_f,key_jump_r,key_jump_e,key_jump_q,key_jump_ctrl,key_jump_shift,key_jump_b;
void USART2_Configuration(void);
extern RC_Ctl_t RC_Ctl;
#endif
