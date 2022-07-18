#include <task_imu.h>
#include <task_led.h>
#include <driver_imu.h>
#include <inv_mpu.h>
#include <driver_filter.h>
#include <user_math.h>
#include <task_vision.h>
#include <task_control.h>
#include <stm32f4xx_it.h>

#define GYRO_SEN 0.00106526443603169529841533860381f

Gyroscope_t g_cloud_gyroscope;
RM_Gyroscope_t g_rm_imu_data;
Filter_t g_chebyshev_filter;	//切比雪夫滤波器

void imu_task(void *pvParameters)
{
//	double q0,q1,q2,q3;
	float usart_count=0;
	//可以等待电机启动至初始化位置
	vTaskDelay(500);
//	bsp_imu_init();
//	//dmp初始化 灯灭之后MPU初始化完成
//	LED3=0;
//	while(mpu_dmp_init());
//	LED3=1;
	
	while(1)
	{
		LED2=!LED2;
		
//		/*RM官方陀螺仪数据*/
//		q0 = g_rm_imu_data.quat_fp32[0];
//		q1 = g_rm_imu_data.quat_fp32[1];
//		q2 = g_rm_imu_data.quat_fp32[2];
//		q3 = g_rm_imu_data.quat_fp32[3];
//		g_rm_imu_data.yaw = atan2(q0 * q3 + q1 * q2, q0 * q0 + q1 * q1 - 0.5) * 57.3f;
//		g_rm_imu_data.pitch = asin(2 * (q0 * q2 - q1 * q3)) * 57.3f;
//		g_rm_imu_data.roll = atan2(q0 * q1 + q2 * q3, q0 * q0 + q3 * q3 - 0.5)* 57.3f;
//		
//		//陀螺仪数据
//		g_rm_imu_data.gx = g_rm_imu_data.gyro_int16[0] ;
//		g_rm_imu_data.gy = g_rm_imu_data.gyro_int16[1] ;
//		g_rm_imu_data.gz = g_rm_imu_data.gyro_int16[2] ;
		
		vision_send_data(0x03,&(g_gimbal_control.yaw_pid_angle.measure),0);
		
		usart_count++;
		if(usart_count == 40) 
		{
			send_rc_data();
			usart_count = 0;
		}
		vTaskDelay(5);
	}
}

