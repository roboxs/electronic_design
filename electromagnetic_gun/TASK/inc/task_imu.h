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


typedef struct
{
	uint8_t quat_euler:1;
	uint8_t gyro_rangle:3;
	uint8_t accel_rangle:2;
	
	int16_t gyro_int16[3];
	int16_t euler_angle[3];//原始
	float euler_angle_fp32[3];//真正欧拉角
	int16_t quat[4];//原始四元数
	float quat_fp32[4];//真正的四元数
	float gyro_sen;//量程
	float accel_sen;//量程
		
	float yaw;
	float pitch;
	float roll;
	float gx;
	float gy;
	float gz;
}RM_Gyroscope_t;

extern Gyroscope_t g_cloud_gyroscope;
extern RM_Gyroscope_t g_rm_imu_data;
void imu_task(void *pvParameters);

#endif

