#include <bsp_nvic.h>

void bsp_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_Structure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//USART6(裁判系统)
	NVIC_Structure.NVIC_IRQChannel					= USART6_IRQn;
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Structure);
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	
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
	
	//Z相 INT 中断
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_Structure.NVIC_IRQChannel                      = EXTI1_IRQn;
    NVIC_Structure.NVIC_IRQChannelCmd                   = ENABLE;
    NVIC_Structure.NVIC_IRQChannelPreemptionPriority    = 4;
    NVIC_Init(&NVIC_Structure);
	
	//TIM4 更新中断
    NVIC_Structure.NVIC_IRQChannel                      = TIM4_IRQn; 
    NVIC_Structure.NVIC_IRQChannelPreemptionPriority    = 1; 
    NVIC_Structure.NVIC_IRQChannelCmd                   = ENABLE;
    NVIC_Init(&NVIC_Structure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	//CAN1->FIFO 0 消息挂起中断
	NVIC_Structure.NVIC_IRQChannel					= CAN1_RX0_IRQn;	
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO 0 消息挂起中断
}
