/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#include <bsp_dma.h>
#include <task_led.h>
#include <JY901.h>
#include <string.h>


/*********************************************************************************************
 *********************************************************************************************/

u16 USART6_RXCnt=0;//����6�������ݵĳ���

struct SGyro 	stcGyro;
struct SAngle 	stcAngle;

void CopeSerialData(u8 ucData)
{
	static unsigned char ucRxBuffer[30];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}

void USART6_DMA_SendData(u16 length)
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7)){}//�ȴ�DMA���Ա�����
	DMA_SetCurrDataCounter(DMA2_Stream7,length);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
														|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//���LISR��	
	DMA_Cmd(DMA2_Stream7,ENABLE);
}

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_IT_TCIF7))//�жϴ������
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
															|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//���LISR��HISR�Ĵ���
	}
}

void USART6_IRQHandler(void)
{
	u8 clear=clear;
//	u8 i=0;
//	if(USART_GetITStatus(USART6,USART_IT_IDLE))
//	{
//		//����жϱ�־λ
//		clear=USART6->SR;
//		clear=USART6->DR;
//		
//		DMA_Cmd(DMA2_Stream2,DISABLE);//��ֹ��������
//		USART6_RXCnt=DMA_JUDGE_RECEIVE_BUF_SIZE-DMA_GetCurrDataCounter(DMA2_Stream2);//�ܵĻ����С��ȥʣ�໺���С
//		DMA_SetCurrDataCounter(DMA2_Stream2,DMA_JUDGE_RECEIVE_BUF_SIZE);//��������DMA�Ĵ�С
//		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2|DMA_FLAG_FEIF2|DMA_FLAG_DMEIF2
//															|DMA_FLAG_TEIF2|DMA_FLAG_HTIF2);//���LISR��HISR�Ĵ���
////		while(USART6_RXCnt--)
////		{
////			g_dma_judge_send_buff[i]=g_dma_judge_receive_buff[i];
////			i++;
////		}
////		USART6_DMA_SendData(i);
//		CopeSerialData(g_dma_judge_receive_buff);
//		DMA_Cmd(DMA2_Stream2,ENABLE);
//		LED4=!LED4;
//	}

	if(USART_GetITStatus(USART6,USART_IT_RXNE))
	{
		CopeSerialData((u8)USART6->DR);//��������
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}



/*********************************************************************************************
 ***********************************�Ӿ��ж����ݴ���******************************************/


void UART8_IRQHandler()
{
	u16 rxcount;
	u8 clear=clear;
	
	if(USART_GetITStatus(UART8,USART_IT_IDLE)!=RESET)
	{
		DMA_Cmd(DMA1_Stream6, DISABLE); 
		clear=UART8->SR;
		clear=UART8->DR;
		rxcount=DMA_VISION_RECEIVE_BUF_SIZE-DMA_GetCurrDataCounter(DMA1_Stream6);//��ǰռ��DMA���ֽ���
		
		while (DMA_GetCmdStatus(DMA1_Stream6));   
		DMA_SetCurrDataCounter(DMA1_Stream6,DMA_VISION_RECEIVE_BUF_SIZE);
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6|DMA_FLAG_HTIF6|DMA_FLAG_TEIF6|DMA_FLAG_DMEIF6|DMA_FLAG_FEIF6); 
		DMA_Cmd(DMA1_Stream6, ENABLE);	
	}
}


//�Ӿ�����
void DMA1_Stream0_IRQHandler(void)  
{  
    if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)!=RESET)
    {   
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        DMA_Cmd(DMA1_Stream0, DISABLE); 
    }  

} 
