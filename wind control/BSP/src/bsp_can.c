#include <bsp_can.h>

void BSP_CAN_Init()
{
	CAN_InitTypeDef 			CAN_InitStructure;
	CAN_FilterInitTypeDef 		CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	
	//CAN1
	CAN_InitStructure.CAN_TTCM			= DISABLE; //Ӳ���Զ��˳����߹ر�״̬
	CAN_InitStructure.CAN_ABOM			= DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_AWUM			= ENABLE; //��һ��������Ϣ������ǰһ����Ϣ
	CAN_InitStructure.CAN_NART			= DISABLE;  //��ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_RFLM			= DISABLE; //��ֹ�����Զ�����ģʽ
	CAN_InitStructure.CAN_TXFP			= DISABLE; //���ȼ�����Ϣ��ʶ��ȷ��
	CAN_InitStructure.CAN_Mode			= CAN_Mode_Normal;//����Ϊ����ģʽ
	CAN_InitStructure.CAN_Prescaler		= 5;//1Mps   45000/(5+3+1)/5
	CAN_InitStructure.CAN_SJW			= CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1			= CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2			= CAN_BS2_3tq;
	CAN_Init(CAN1,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber			= 0;//������0
	CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FilterFIFO0;//������0������FIFO 0
	CAN_FilterInitStructure.CAN_FilterScale				= CAN_FilterScale_16bit;//32�Ĵ���
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 0x0000;	//��ʶ���Ĵ���
	CAN_FilterInitStructure.CAN_FilterIdLow				= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0x0000;	//���μĴ���
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			= 0x0000; 
	CAN_FilterInitStructure.CAN_FilterMode				= CAN_FilterMode_IdMask;//��ʶ������ģʽ
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//CAN2
	CAN_InitStructure.CAN_ABOM			= ENABLE; //Ӳ���Զ��˳����߹ر�״̬
	CAN_InitStructure.CAN_AWUM			= DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_RFLM			= DISABLE;//��һ��������Ϣ������ǰһ����Ϣ
	CAN_InitStructure.CAN_TTCM			= DISABLE;//��ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_NART			= DISABLE;//��ֹ�����Զ�����ģʽ
	CAN_InitStructure.CAN_TXFP			= DISABLE;//���ȼ�����Ϣ��ʶ��ȷ��
	CAN_InitStructure.CAN_Mode			= CAN_Mode_Normal;//����Ϊ����ģʽ
	CAN_InitStructure.CAN_Prescaler		= 5;//1Mps
	CAN_InitStructure.CAN_SJW			= CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1			= CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2			= CAN_BS2_3tq;
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber			= 14;//������14
	CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FilterFIFO1;//������10������FIFO 1
	CAN_FilterInitStructure.CAN_FilterScale				= CAN_FilterScale_32bit;//32�Ĵ���
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 0x0000;		//��ʶ���Ĵ���
	CAN_FilterInitStructure.CAN_FilterIdLow				= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0x0000;	//���μĴ���
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			= 0x0000; 
	CAN_FilterInitStructure.CAN_FilterMode				= CAN_FilterMode_IdMask;//��ʶ������ģʽ
	CAN_FilterInit(&CAN_FilterInitStructure);
	
}

