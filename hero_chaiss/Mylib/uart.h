#ifndef __USART_H__
#define __USART_H__
#include "stm32f10x.h"
void USART1_Configuration(void);
void USART1_SendChar(char b);
void USART1_IRQHandler(void);
void USART1_Send_Date( USART_TypeDef* USARTx, u8 *pWord, u16 Len );
#endif
