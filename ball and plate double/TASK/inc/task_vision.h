#ifndef _TASK_VISION_H
#define _TASK_VISION_H
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f4xx.h>

#define DN_REG_ID 0xA5 //帧头




typedef struct
{
	float get_target_angle_x;
	float last_get_target_angle_x;
	float get_target_angle_y;
	float last_get_target_angle_y;
	
	float send_motor_angle_x;
	float send_motor_angle_y;
	
	float xspeed;
	float yspeed;
	float after_xspeed;
	float after_yspeed;
}VisionData_t;


void vision_task(void *pvParameters);
void vision_update_data(void);
void vision_receive_data(void);
void vision_send_data(u8 cmd , float *yaw, float *pitch);

extern TaskHandle_t xHandleTaskPCParse;
extern VisionData_t g_vision_data;
#endif
