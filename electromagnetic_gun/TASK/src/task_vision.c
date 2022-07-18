#include <FreeRTOS.h>
#include <task.h>
#include <task_vision.h>
#include <bsp_dma.h>
#include <user_math.h>
#include <task_imu.h>
#include <driver_crc.h>

TaskHandle_t xHandleTaskPCParse;
VisionData_t g_vision_data;
BluetoothData_t g_bluetooth_data;
static HexToFloat_t vision_memory;
static HexToFloat_t vision_send_memory;

void vision_task(void *pvParameters)
{

	while(1)
	{
		bluetooth_data_handle(g_dma_judge_receive_buff);
		vision_receive_data();
		vTaskDelay(5);
	}
}

void vision_receive_data(void)
{
	//更新数据
	g_vision_data.last_get_target_angle_yaw = g_vision_data.get_target_angle_yaw;
	g_vision_data.get_last_distance = g_vision_data.get_current_distance;
	
	if(g_dma_vision_receice_buff[0] == DN_REG_ID && g_dma_vision_receice_buff[1] == 0X03&&
		verify_crc8_check_sum(g_dma_vision_receice_buff,11))
	{
		//x轴数据处理
		vision_memory.hex_data[3] = g_dma_vision_receice_buff[5];
		vision_memory.hex_data[2] = g_dma_vision_receice_buff[4];
		vision_memory.hex_data[1] = g_dma_vision_receice_buff[3];
		vision_memory.hex_data[0] = g_dma_vision_receice_buff[2];
		g_vision_data.get_target_angle_yaw = vision_memory.float_data;
		//y轴数据处理
		vision_memory.hex_data[3] = g_dma_vision_receice_buff[9];
		vision_memory.hex_data[2] = g_dma_vision_receice_buff[8];
		vision_memory.hex_data[1] = g_dma_vision_receice_buff[7];
		vision_memory.hex_data[0] = g_dma_vision_receice_buff[6];
		g_vision_data.get_current_distance = vision_memory.float_data;
	}
//	//噪声处理
//	if(ABS(g_vision_data.get_target_angle_yaw - g_vision_data.last_get_target_angle_yaw) > 8.0f)
//	{
//		g_vision_data.get_target_angle_yaw = g_vision_data.last_get_target_angle_yaw;
//	}
//	g_vision_data.xspeed = g_vision_data.get_target_angle_x - g_vision_data.last_get_target_angle_x;
//	g_vision_data.yspeed = g_vision_data.get_target_angle_y - g_vision_data.last_get_target_angle_y;
}

void vision_send_data(u8 cmd , float *yaw, float *pitch)
{
	u8 dwLength;
	
	if(cmd==0X03)   //发送云台角度数据
	{
		dwLength=11;
		g_dma_vision_send_buff[0]=0XA6;
		g_dma_vision_send_buff[1]=cmd;
		
		vision_send_memory.float_data= *pitch;
		g_dma_vision_send_buff[2] = vision_send_memory.hex_data[0];
		g_dma_vision_send_buff[3] = vision_send_memory.hex_data[1];
		g_dma_vision_send_buff[4] = vision_send_memory.hex_data[2];
		g_dma_vision_send_buff[5] = vision_send_memory.hex_data[3];
		
        vision_send_memory.float_data= *yaw;
		g_dma_vision_send_buff[6]= vision_send_memory.hex_data[0];
		g_dma_vision_send_buff[7]= vision_send_memory.hex_data[1];
		g_dma_vision_send_buff[8]= vision_send_memory.hex_data[2];
		g_dma_vision_send_buff[9]= vision_send_memory.hex_data[3];
		
		
		append_crc8_check_sum(g_dma_vision_send_buff,2+4+4+1);
	}

	DMA_Cmd(DMA1_Stream0, DISABLE);
	while (DMA_GetCmdStatus(DMA1_Stream0)); 
	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0|DMA_FLAG_HTIF0|DMA_FLAG_TEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_FEIF0);                                                                                                 
	DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_FEIF0);
	DMA_SetCurrDataCounter(DMA1_Stream0,dwLength);
	DMA_Cmd(DMA1_Stream0, ENABLE);
}

void bluetooth_data_handle(u8 * data)
{
	if(data[0]==0XA5)
	{
		g_bluetooth_data.mode = data[1];
		g_bluetooth_data.alpha_short = data[3]<<8|data[2];
		g_bluetooth_data.d = data[5]<<8|data[4];
	}
	else if(data[0] == 0xA6)
	{
		g_bluetooth_data.cmd = data[1];
	}
	
	
	g_bluetooth_data.alpha = (float)g_bluetooth_data.alpha_short /10;
	//距离限制
	if(g_bluetooth_data.d < 200)g_bluetooth_data.d = 0;
	else if(g_bluetooth_data.d > 300)g_bluetooth_data.d =300;
	//角度限制
	if(g_bluetooth_data.alpha < -30)g_bluetooth_data.alpha =-30;
	else if(g_bluetooth_data.alpha > 30)g_bluetooth_data.alpha = 30;
}

