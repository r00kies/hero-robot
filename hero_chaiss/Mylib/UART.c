#include "main.h"
#define Uart1_Rx_LEN		128
uint8_t Uart1_Rx[Uart1_Rx_LEN];
uint8_t com_cmd1_flag=0,com_cmd2_flag=0;
void USART1_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA,&gpio);

    gpio.GPIO_Pin = GPIO_Pin_9;  
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&gpio);

    USART_DeInit(USART1);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No ;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_Init(USART1,&usart);
		
		 USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);   
		 USART_ITConfig(USART1, USART_IT_RXNE ,DISABLE);
     USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	   USART_Cmd(USART1,ENABLE);
    
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    dma.DMA_MemoryBaseAddr = (uint32_t)Uart1_Rx;   
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = Uart1_Rx_LEN;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5,&dma);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
		
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		
   
   nvic.NVIC_IRQChannel = USART1_IRQn;
   nvic.NVIC_IRQChannelPreemptionPriority = 2;
   nvic.NVIC_IRQChannelSubPriority = 1;
   nvic.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&nvic); 
   DMA_Cmd(DMA1_Channel5, ENABLE);	 
   
}

void USART1_SendChar(char b)
{
    while( USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	USART_SendData(USART1,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

void DMA1_Channel5_IRQHandler() 
{
    if(DMA_GetITStatus(DMA1_IT_TC5) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC5);
        DMA_ClearITPendingBit(DMA1_IT_TC5);
    }
}
void USART1_Send_Date( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
{
	u8 i = 0;
	for(i=0; i<Len; i++) {
		USART_SendData(USARTx, (uint8_t)*pWord);
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC) == RESET);
		pWord++;
	}
}

void USART1_IRQHandler(void)                               
{
	uint32_t temp = 0,i;
	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
			TIM_Cmd(TIM4, DISABLE);
    	temp = USART1->SR;
    	temp = USART1->DR;
    	DMA_Cmd(DMA1_Channel5,DISABLE);
	  	temp = Uart1_Rx_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
		for(i=0;i<temp;i++)
    ANO_DT_Data_Receive_Prepare(Uart1_Rx[i]);
		DMA_SetCurrDataCounter(DMA1_Channel5,Uart1_Rx_LEN);
		DMA_Cmd(DMA1_Channel5,ENABLE);
			TIM_Cmd(TIM4, ENABLE);
    } 
	__nop(); 
} 
