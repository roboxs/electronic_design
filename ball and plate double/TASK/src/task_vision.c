#include <FreeRTOS.h>
#include <task.h>
#include <task_vision.h>
#include <bsp_dma.h>
#include <user_math.h>
#include <task_imu.h>
#include <driver_filter.h>

TaskHandle_t xHandleTaskPCParse;
VisionData_t g_vision_data;
static HexToFloat_t vision_memory;
static HexToFloat_t vision_send_memory;
LowFilter_s g_xspeed_low_filter;
LowFilter_s g_yspeed_low_filter;

void vision_task(void *pvParameters)
{

	while(1)
	{
		vision_receive_data();
		g_xspeed_low_filter.yi = g_vision_data.xspeed;
		g_yspeed_low_filter.yi = g_vision_data.yspeed;
		first_order_low_pass_filter(&g_xspeed_low_filter);
		first_order_low_pass_filter(&g_yspeed_low_filter);
		g_vision_data.after_xspeed = g_xspeed_low_filter.yo;
		g_vision_data.after_yspeed = g_yspeed_low_filter.yo;
		vTaskDelay(15);
	}
}

void vision_receive_data(void)
{
	//更新数据
	g_vision_data.last_get_target_angle_x = g_vision_data.get_target_angle_x;
	g_vision_data.last_get_target_angle_y = g_vision_data.get_target_angle_y;
	//x轴数据处理
	vision_memory.hex_data[3] = g_dma_vision_receice_buff[5];
	vision_memory.hex_data[2] = g_dma_vision_receice_buff[4];
	vision_memory.hex_data[1] = g_dma_vision_receice_buff[3];
	vision_memory.hex_data[0] = g_dma_vision_receice_buff[2];
	g_vision_data.get_target_angle_x = vision_memory.float_data;
	//y轴数据处理
	vision_memory.hex_data[3] = g_dma_vision_receice_buff[9];
	vision_memory.hex_data[2] = g_dma_vision_receice_buff[8];
	vision_memory.hex_data[1] = g_dma_vision_receice_buff[7];
	vision_memory.hex_data[0] = g_dma_vision_receice_buff[6];
	g_vision_data.get_target_angle_y = vision_memory.float_data;
	//噪声处理
	if(ABS(g_vision_data.get_target_angle_x - g_vision_data.last_get_target_angle_x ) > 100.0f)
		g_vision_data.get_target_angle_x = g_vision_data.last_get_target_angle_x;
	if(ABS(g_vision_data.get_target_angle_y - g_vision_data.last_get_target_angle_y ) > 100.0f)
		g_vision_data.get_target_angle_y = g_vision_data.last_get_target_angle_y;
	
	g_vision_data.xspeed = g_vision_data.get_target_angle_x - g_vision_data.last_get_target_angle_x;
	g_vision_data.yspeed = g_vision_data.get_target_angle_y - g_vision_data.last_get_target_angle_y;
}

void vision_send_data(u8 cmd , float *yaw, float *pitch)
{
	u8 dwLength;
	if(cmd==0x01)   //发送目标装甲大小信息
	{
		dwLength=4;
		g_dma_vision_send_buff[0]=0XA6;
		g_dma_vision_send_buff[1]=cmd;
	}

	DMA_Cmd(DMA1_Stream0, DISABLE);
	while (DMA_GetCmdStatus(DMA1_Stream0)); 
	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0|DMA_FLAG_HTIF0|DMA_FLAG_TEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_FEIF0);                                                                                                 
	DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_FEIF0);
	DMA_SetCurrDataCounter(DMA1_Stream0,dwLength);
	DMA_Cmd(DMA1_Stream0, ENABLE);
}

