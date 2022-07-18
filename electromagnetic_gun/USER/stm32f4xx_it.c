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
#include <user_math.h>
#include <stdio.h>
#

#define GUN_DISTANCE 0.082

void send_rc_data(void);

uint64_t Get_SysTimeUs(void)
{
    uint64_t ms;
    uint64_t value;
    ms = xTaskGetTickCount();
    value = ms * 1000 + (SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD;
    return value;
}

/*********************************************************************************************
 *********************************************************************************************/
unsigned	int g_time3_count;
unsigned	int g_first_time;
unsigned	int g_second_time;
short g_time_error;
float g_speed_bull;
void EXTI0_IRQHandler(void)
{
	g_second_time = Get_SysTimeUs();
//	g_second_time = g_time3_count;
	g_time_error =g_second_time - g_first_time;
	g_speed_bull = GUN_DISTANCE / g_time_error * 1000000;//单位是M/S
	g_time3_count = 0;
//	send_rc_data();
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位 
}

void EXTI1_IRQHandler(void)
{
	g_first_time = Get_SysTimeUs();
//	g_first_time = g_time3_count;
	EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE0上的中断标志位 
}

/*********************************************************************************************
 *********************************************************************************************/

u16 USART6_RXCnt=0;//串口6接受数据的长度


void USART6_DMA_SendData(u16 length)
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7)){}//等待DMA可以被设置
	DMA_SetCurrDataCounter(DMA2_Stream7,length);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
														|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//清除LISR和	
	DMA_Cmd(DMA2_Stream7,ENABLE);
}

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_IT_TCIF7))//判断传输完成
	{
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7
															|DMA_FLAG_TEIF7|DMA_FLAG_HTIF7);//清除LISR和HISR寄存器
	}
}


void USART6_IRQHandler(void)
{
	u8 clear=clear;
	u8 i=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE))
	{
		//清除中断标志位
		clear=USART6->SR;
		clear=USART6->DR;
		
		DMA_Cmd(DMA2_Stream2,DISABLE);//防止传输数据
		USART6_RXCnt=DMA_JUDGE_RECEIVE_BUF_SIZE-DMA_GetCurrDataCounter(DMA2_Stream2);//总的缓存大小减去剩余缓存大小
		DMA_SetCurrDataCounter(DMA2_Stream2,DMA_JUDGE_RECEIVE_BUF_SIZE);//重新设置DMA的大小
		DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2|DMA_FLAG_FEIF2|DMA_FLAG_DMEIF2
															|DMA_FLAG_TEIF2|DMA_FLAG_HTIF2);//清除LISR和HISR寄存器
		while(USART6_RXCnt--)
		{
			g_dma_judge_send_buff[i]=g_dma_judge_receive_buff[i];
			i++;
		}
		USART6_DMA_SendData(i);
		DMA_Cmd(DMA2_Stream2,ENABLE);
		LED4=!LED4;
	}
}

void send_rc_data(void)
{
	u8 i=0;
	sprintf(g_dma_judge_send_buff,"page0.t3.txt=\"%6.2f\"",g_speed_bull);
	
	for(i=0;i<50;i++)
	{
		if((g_dma_judge_send_buff[i]=='"')&&(g_dma_judge_send_buff[i-1]!='='))
			break;
	}
	g_dma_judge_send_buff[i+1]=0xff;
	g_dma_judge_send_buff[i+2]=0xff;
	g_dma_judge_send_buff[i+3]=0xff;
	
	
	DMA_Cmd(DMA2_Stream7, DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7)); 
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_HTIF7|DMA_FLAG_TEIF7|DMA_FLAG_DMEIF7|DMA_FLAG_FEIF7);                                                                                                 
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TEIF7|DMA_IT_HTIF7|DMA_IT_TCIF7|DMA_IT_DMEIF7|DMA_IT_FEIF7);
	DMA_SetCurrDataCounter(DMA2_Stream7,i+3+1);
	DMA_Cmd(DMA2_Stream7, ENABLE);
}



/*********************************************************************************************
 ***********************************视觉中断数据处理******************************************/

void UART8_IRQHandler()
{
	u16 rxcount;
	u8 clear=clear;
	
	if(USART_GetITStatus(UART8,USART_IT_IDLE)!=RESET)
	{
		DMA_Cmd(DMA1_Stream6, DISABLE); 
		clear=UART8->SR;
		clear=UART8->DR;
		rxcount=DMA_VISION_RECEIVE_BUF_SIZE-DMA_GetCurrDataCounter(DMA1_Stream6);//当前占用DMA的字节数
		
		while (DMA_GetCmdStatus(DMA1_Stream6));   
		DMA_SetCurrDataCounter(DMA1_Stream6,DMA_VISION_RECEIVE_BUF_SIZE);
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6|DMA_FLAG_HTIF6|DMA_FLAG_TEIF6|DMA_FLAG_DMEIF6|DMA_FLAG_FEIF6); 
		DMA_Cmd(DMA1_Stream6, ENABLE);	
	}
}


//视觉发送
void DMA1_Stream0_IRQHandler(void)  
{  
    if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)!=RESET)
    {   
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        DMA_Cmd(DMA1_Stream0, DISABLE); 
    }  

} 


/*********************************************************************************************
 *********************************************************************************************/
#include <task_imu.h>
#include <string.h>

#define RM_IMU_QUAT_ID 0x401
#define RM_IMU_GYRO_ID 0x402
#define RM_IMU_ACCEL_ID 0x403
#define RM_IMU_MAG_ID 0x404
#define RM_IMU_PARAM_ID 0x405

//转换成 m/s^2
#define ACCEL_3G_SEN 0.0008974358974f
#define ACCEL_6G_SEN 0.00179443359375f
#define ACCEL_12G_SEN 0.0035888671875f
#define ACCEL_24G_SEN 0.007177734375f
//转换成 rad/s
#define GYRO_2000_SEN 0.00106526443603169529841533860381f
#define GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define GYRO_500_SEN 0.00026631610900792382460383465095346f
#define GYRO_250_SEN 0.00013315805450396191230191732547673f
#define GYRO_125_SEN 0.000066579027251980956150958662738366f

CanRxMsg g_can2_receive_str;

void CAN2_RX1_IRQHandler(void)
{
	CAN_Receive(CAN2,CAN_FIFO1,&g_can2_receive_str);
	switch(g_can2_receive_str.StdId)
	{
		case RM_IMU_PARAM_ID:
		{
			g_rm_imu_data.accel_rangle = g_can2_receive_str.Data[0] &0x0F;
			g_rm_imu_data.gyro_rangle = (g_can2_receive_str.Data[0] &0xF0) >> 4;
			switch(g_rm_imu_data.gyro_rangle)
			{
				case 0: g_rm_imu_data.gyro_sen = GYRO_2000_SEN; break;
				case 1: g_rm_imu_data.gyro_sen = GYRO_1000_SEN; break;
				case 2: g_rm_imu_data.gyro_sen = GYRO_500_SEN; break;
				case 3: g_rm_imu_data.gyro_sen = GYRO_250_SEN; break;
				case 4: g_rm_imu_data.gyro_sen = GYRO_125_SEN; break;
			}
			switch(g_rm_imu_data.accel_rangle)
			{
				case 0: g_rm_imu_data.accel_sen = ACCEL_3G_SEN; break;
				case 1: g_rm_imu_data.accel_sen = ACCEL_6G_SEN; break;
				case 2: g_rm_imu_data.accel_sen = ACCEL_12G_SEN; break;
				case 3: g_rm_imu_data.accel_sen = ACCEL_24G_SEN; break;
			}
			break;
		}
		case RM_IMU_QUAT_ID:
		{
			if(g_rm_imu_data.quat_euler && g_can2_receive_str.DLC == 6)
			{
				memcpy(g_rm_imu_data.euler_angle, g_can2_receive_str.Data, g_can2_receive_str.DLC);
				g_rm_imu_data.euler_angle_fp32[0] = g_rm_imu_data.euler_angle[0] * 0.0001f;
				g_rm_imu_data.euler_angle_fp32[1] = g_rm_imu_data.euler_angle[1] * 0.0001f;
				g_rm_imu_data.euler_angle_fp32[2] = g_rm_imu_data.euler_angle[2] * 0.0001f;
			}
			else if(g_rm_imu_data.quat_euler == 0 && g_can2_receive_str.DLC == 8)
			{
				memcpy(g_rm_imu_data.quat, g_can2_receive_str.Data, g_can2_receive_str.DLC);
				g_rm_imu_data.quat_fp32[0] = g_rm_imu_data.quat[0] * 0.0001f;
				g_rm_imu_data.quat_fp32[1] = g_rm_imu_data.quat[1] * 0.0001f;
				g_rm_imu_data.quat_fp32[2] = g_rm_imu_data.quat[2] * 0.0001f;
				g_rm_imu_data.quat_fp32[3] = g_rm_imu_data.quat[3] * 0.0001f;
			}
			break;
		}
		case RM_IMU_GYRO_ID:
		{
			memcpy(g_rm_imu_data.gyro_int16, g_can2_receive_str.Data,6);
//			g_rm_imu_data.sensor_temperature = (int16_t)((rx_message->Data[6] << 3) | (rx_message->Data[7] >>
//			5));
//			if (g_rm_imu_data.sensor_temperature > 1023)
//			{
//				g_rm_imu_data.sensor_temperature -= 2048;
//			}
			break;
		}
	}
}

/*********************************************************************************************
 *********************************************************************************************/
#include <driver_encoder.h>

#define RM_MOTOR_YAW 0x205
#define RM_MOTOR_PITCH 0x206

CanRxMsg g_can1_receive_str;
MotoMeasure_t moto6020_yaw, moto6020_pitch;

//返回yaw电机的指针
const MotoMeasure_t * get_moto6020_yaw_point(void)
{
	return &moto6020_yaw;
}
//返回pitch电机的指针
const MotoMeasure_t * get_moto6020_pitch_point(void)
{
	return &moto6020_pitch;
}

void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}


void CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0,&g_can1_receive_str);
	switch(g_can1_receive_str.StdId)
	{
		case RM_MOTOR_YAW:
			encoder_data_handler(&moto6020_yaw,&g_can1_receive_str);
			break;
		case RM_MOTOR_PITCH:
			encoder_data_handler(&moto6020_pitch,&g_can1_receive_str);
			break;
	}
}


/*********************************************************************************************
 *********************************************************************************************/
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
//		LED5=!LED5;
//		g_time3_count ++;
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}

