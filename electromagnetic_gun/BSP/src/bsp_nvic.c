#include <bsp_nvic.h>

void bsp_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_Structure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
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
	
	//CAN1->FIFO 0 消息挂起中断
	NVIC_Structure.NVIC_IRQChannel					= CAN1_RX0_IRQn;	
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO 0 消息挂起中断
	
	//CAN2->FIF0 1 消息挂起中断
	NVIC_Structure.NVIC_IRQChannel					= CAN2_RX1_IRQn;
	NVIC_Structure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 11;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO 1消息挂起中断
	
	//使能外部中断
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);//PA0 连接到中断线0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);//PA1 连接到中断线1
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_Structure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_Structure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_Structure);
	
	NVIC_Structure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断2
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级3
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_Structure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_Structure);
	
	
//	NVIC_Structure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
//	NVIC_Structure.NVIC_IRQChannelPreemptionPriority= 0x03; //抢占优先级1
//	NVIC_Structure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_Structure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_Structure);
}
