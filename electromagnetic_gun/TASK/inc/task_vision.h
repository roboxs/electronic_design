#ifndef _TASK_VISION_H
#define _TASK_VISION_H
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f4xx.h>

#define DN_REG_ID 0xA5 //帧头




typedef struct
{
	float get_target_angle_yaw;
	float last_get_target_angle_yaw;
	float get_current_distance;
	float get_last_distance;
	
	float send_motor_angle_x;
	float send_motor_angle_y;
	
	float xspeed;
	float yspeed;
}VisionData_t;

typedef struct
{
	u8 mode;
	u8 cmd;
	short alpha_short;
	float alpha;
	short d;
}BluetoothData_t;


void vision_task(void *pvParameters);
void vision_update_data(void);
void vision_receive_data(void);
void vision_send_data(u8 cmd , float *yaw, float *pitch);
void bluetooth_data_handle(u8 * data);

extern TaskHandle_t xHandleTaskPCParse;
extern VisionData_t g_vision_data;
extern BluetoothData_t g_bluetooth_data;
#endif
