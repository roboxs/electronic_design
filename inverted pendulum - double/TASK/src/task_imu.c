#include <task_imu.h>
#include <task_led.h>
#include <driver_imu.h>
#include <inv_mpu.h>
#include <driver_filter.h>

#define GYRO_SEN 0.00106526443603169529841533860381f

Gyroscope_t g_cloud_gyroscope;
Filter_t g_chebyshev_filter;	//切比雪夫滤波器

void imu_task(void *pvParameters)
{
	float pitch, roll, yaw;
	short raw_gx,raw_gy,raw_gz;
	
	//可以等待电机启动至初始化位置
	vTaskDelay(3000);
	bsp_imu_init();
	//dmp初始化 灯灭之后MPU初始化完成
	LED3=0;
	while(mpu_dmp_init());
	LED3=1;
	
	
	while(1)
	{
		LED2=~LED2;
		//获取欧拉角
		mpu_mpl_get_data(&pitch, &roll, &yaw);
		//获取陀螺仪角速度原始数据
		MPU_Get_Gyroscope(&raw_gx, &raw_gy, &raw_gz);
		g_cloud_gyroscope.pitch = pitch;
		g_cloud_gyroscope.roll  = roll;
		g_cloud_gyroscope.yaw   = yaw;
		
		//对原数据进行滤波处理
//		g_chebyshev_filter.raw_value= -raw_gx;
//		Chebyshev50HzLPF(&g_chebyshev_filter);
//		g_cloud_gyroscope.gx=g_chebyshev_filter.filtered_value;
		g_cloud_gyroscope.gx=-raw_gx;//PITCH
		g_cloud_gyroscope.gy=-raw_gy;
		g_cloud_gyroscope.gz=-raw_gz;//YAW
		
		//vision_send_data(0x02 , &g_cloud_gyroscope.real_yaw, &g_gimbal_control.motor_pitch.angle_measure);
		vTaskDelay(5);
	}
}

