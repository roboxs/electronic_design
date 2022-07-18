#include <bsp_can.h>

void BSP_CAN_Init()
{
	CAN_InitTypeDef 			CAN_InitStructure;
	CAN_FilterInitTypeDef 		CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	
	//CAN1
	CAN_InitStructure.CAN_TTCM			= DISABLE; //硬件自动退出总线关闭状态
	CAN_InitStructure.CAN_ABOM			= DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_AWUM			= ENABLE; //下一条传入消息将覆盖前一条消息
	CAN_InitStructure.CAN_NART			= DISABLE;  //非时间触发通信模式
	CAN_InitStructure.CAN_RFLM			= DISABLE; //禁止报文自动发送模式
	CAN_InitStructure.CAN_TXFP			= DISABLE; //优先级由消息标识符确定
	CAN_InitStructure.CAN_Mode			= CAN_Mode_Normal;//设置为正常模式
	CAN_InitStructure.CAN_Prescaler		= 5;//1Mps   45000/(5+3+1)/5
	CAN_InitStructure.CAN_SJW			= CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1			= CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2			= CAN_BS2_3tq;
	CAN_Init(CAN1,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber			= 0;//过滤器0
	CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FilterFIFO0;//过滤器0关联到FIFO 0
	CAN_FilterInitStructure.CAN_FilterScale				= CAN_FilterScale_16bit;//32寄存器
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 0x0000;	//标识符寄存器
	CAN_FilterInitStructure.CAN_FilterIdLow				= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0x0000;	//屏蔽寄存器
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			= 0x0000; 
	CAN_FilterInitStructure.CAN_FilterMode				= CAN_FilterMode_IdMask;//标识符屏蔽模式
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//CAN2
	CAN_InitStructure.CAN_ABOM			= ENABLE; //硬件自动退出总线关闭状态
	CAN_InitStructure.CAN_AWUM			= DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_RFLM			= DISABLE;//下一条传入消息将覆盖前一条消息
	CAN_InitStructure.CAN_TTCM			= DISABLE;//非时间触发通信模式
	CAN_InitStructure.CAN_NART			= DISABLE;//禁止报文自动发送模式
	CAN_InitStructure.CAN_TXFP			= DISABLE;//优先级由消息标识符确定
	CAN_InitStructure.CAN_Mode			= CAN_Mode_Normal;//设置为环回模式
	CAN_InitStructure.CAN_Prescaler		= 5;//1Mps
	CAN_InitStructure.CAN_SJW			= CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1			= CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2			= CAN_BS2_3tq;
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber			= 14;//过滤器14
	CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FilterFIFO1;//过滤器10关联到FIFO 1
	CAN_FilterInitStructure.CAN_FilterScale				= CAN_FilterScale_32bit;//32寄存器
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 0x0000;		//标识符寄存器
	CAN_FilterInitStructure.CAN_FilterIdLow				= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0x0000;	//屏蔽寄存器
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			= 0x0000; 
	CAN_FilterInitStructure.CAN_FilterMode				= CAN_FilterMode_IdMask;//标识符屏蔽模式
	CAN_FilterInit(&CAN_FilterInitStructure);
	
}

