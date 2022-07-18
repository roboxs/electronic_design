#include <bsp_nvic.h>

void bsp_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_Structure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//USART6(裁判系统)
	NVIC_Structure.NVIC_IRQChannel					= USART6_IRQn;
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Structure);
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	
	//USART6->DMA发送完成中断
	NVIC_Structure.NVIC_IRQChannel 					= DMA2_Stream7_IRQn;  	
	NVIC_Structure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;  
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_Init(&NVIC_Structure);  
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	
	//UART8->DMA接收空闲中断
    NVIC_Structure.NVIC_IRQChannel                    = UART8_IRQn;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority  = 3;
	NVIC_Structure.NVIC_IRQChannelCmd                 = ENABLE;			
	NVIC_Init(&NVIC_Structure);	
    USART_ITConfig(UART8, USART_IT_IDLE, ENABLE);
    
    //UART8->DMA发送完成中断
    NVIC_Structure.NVIC_IRQChannel                     = DMA1_Stream0_IRQn;
    NVIC_Structure.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Structure.NVIC_IRQChannelPreemptionPriority   = 8;
    NVIC_Init(&NVIC_Structure);
    DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF7);
    DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
}
