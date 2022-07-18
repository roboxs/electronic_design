#ifndef _TASK_IMU_H
#define _TASK_IMU_H

#include <stm32f4xx.h>


typedef struct
{
	float pitch;
	float yaw;
	float roll;
	float last_deal_yaw;
	float deal_yaw;
	float real_yaw;
	float rount;
	
	short gx;
	short gy;
	short gz;
}Gyroscope_t;


extern Gyroscope_t g_cloud_gyroscope;

void imu_task(void *pvParameters);

#endif

