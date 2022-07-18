#include <task_control.h>
#include <FreeRTOS.h>
#include <task.h>
#include <task_led.h>
#include <driver_control.h>
#include <user_math.h>
#include <driver_motor.h>
#include <task_imu.h>
#include <math.h>
#include <stm32f4xx_it.h>
#include <bsp_dma.h>

WindControl_t g_wind_control;

float g_kp_y =-10.0f,  g_ki_y = 0.0f, g_kd_y =0.0f;
float g_kp_x =10.0f,  g_ki_x = 0.0f, g_kd_x =0.0f;
float g_kp_sy =150.0f,  g_ki_sy = 0.1f, g_kd_sy =0.0f;
float g_kp_sx =150.0f,  g_ki_sx = 0.1f, g_kd_sx =0.0f;
float g_maxout_x= 2000.0f,g_maxout_y= 2000.0f;
float g_int_limit_x = 1000.0f, g_int_limit_y = 1000.0f;
float g_target_x = 0.0f,g_target_y = 0.0f;
//float g_deadband = 0.3f;
float test_targetx = 0;
float test_targety = 0;
float g_time = 2000;

float g_line_target = 10.0f;
float g_line_angle = 1.0f;
float g_cycle_target  = 10.0f;
void control_task(void *pvParameters)
{
//	float moto_xpluse = 0.05 * 20000;
	vTaskDelay(5000);//等待电调启动
	control_task_init();
	while(1)
	{
		//风力摆模式设置
		wind_set_mode(&g_wind_control);
		//电机x,y方向角度值
		set_wind_target(&g_wind_control);
		//任务数据更新 在IMU任务中进行更新
		task_control_update(&g_wind_control);
		//pid控制更新
		control_pid_calculate(&g_wind_control);
		//给电机赋值
//		driver_wind_motor(test_targetx,g_wind_control.wind_pid_yspeed.pos_out);
//		driver_wind_motor(g_wind_control.wind_pid_xspeed.pos_out,test_targety);
//		driver_wind_motor(test_targetx,test_targety);
		
		if(g_wind_control.mode == WIND_CLOSE)
		{
			driver_wind_motor(0 , 0);
		}
		else 
		{
			driver_wind_motor(g_wind_control.wind_pid_xspeed.pos_out,g_wind_control.wind_pid_yspeed.pos_out);
		}
		
//		/*测试添加代码*/	
//		/*x轴*/
//		pid_reset(&g_wind_control.wind_pid_xspeed,g_kp_sx,g_ki_sx,g_kd_sx);
//		pid_reset(&g_wind_control.wind_pid_xangle,g_kp_x,g_ki_x,g_kd_x);
//		g_wind_control.wind_pid_xangle.MaxOutput = g_maxout_x;
//		//g_wind_control.wind_pid_xspeed.IntegralLimit = g_int_limit_x;
//		
//		/*y轴*/
//		pid_reset(&g_wind_control.wind_pid_yspeed,g_kp_sy,g_ki_sy,g_kd_sy);
//		pid_reset(&g_wind_control.wind_pid_yangle,g_kp_y,g_ki_y,g_kd_y);
//		g_wind_control.wind_pid_yangle.MaxOutput = g_maxout_y;
////		g_wind_control.wind_pid_yspeed.IntegralLimit = g_int_limit_y;
		
		LED5=!LED5;
		vTaskDelay(10);
	}
}


static void control_task_init(void)
{
//电机双环控制
/*x方向速度环*/	pid_struct_init(&g_wind_control.wind_pid_xspeed,POSITION_PID,DOUBLE_LOOP,WIND_XSPEED_MAXOUT, WIND_XSPEED_INTEGRATION_LIMIT,
					WIND_XSPEED_KP,WIND_XSPEED_KI,WIND_XSPEED_KD);
/*x方向角度环*/	pid_struct_init(&g_wind_control.wind_pid_xangle,POSITION_PID,DOUBLE_LOOP,WIND_XANGLE_MAXOUT,WIND_XANGLE_INTEGRATION_LIMIT,
					WIND_XANGLE_KP,WIND_XANGLE_KI,WIND_XANGLE_KD);
/*y方向速度环*/	pid_struct_init(&g_wind_control.wind_pid_yspeed,POSITION_PID,DOUBLE_LOOP,WIND_YSPEED_MAXOUT, WIND_YSPEED_INTEGRATION_LIMIT,
					WIND_YSPEED_KP,WIND_YSPEED_KI,WIND_YSPEED_KD);
/*y方向角度环*/	pid_struct_init(&g_wind_control.wind_pid_yangle,POSITION_PID,DOUBLE_LOOP,WIND_YANGLE_MAXOUT,WIND_YANGLE_INTEGRATION_LIMIT,
					WIND_YANGLE_KP,WIND_YANGLE_KI,WIND_YANGLE_KD);
	//设置电机的死区
	g_wind_control.wind_pid_xangle.deadband = 0.0f;
	g_wind_control.wind_pid_yangle.deadband = 0.0f;
	
	//获取初始时角度的偏差
	g_wind_control.offset_xangle = (float)stcAngle.Angle[0]/32768*180;
	g_wind_control.offset_yangle = (float)stcAngle.Angle[1]/32768*180;
	
//	g_wind_control.mode = WIND_CYCLE;
//	g_wind_control.cycle_r = 8.0f;
}
/*蓝牙与该函数通信即可*/
static void wind_set_mode(WindControl_t *wind_mode)
{
	if(g_dma_vision_receice_buff[0] == 0X5A && (g_dma_vision_receice_buff[1] == 0X01 || g_dma_vision_receice_buff[1] == 0X02) )
	{
		wind_mode->mode = WIND_LINE;
		wind_mode->line_length = linearizaiton(g_dma_vision_receice_buff[2],30,60,8,10);
		wind_mode->line_angle  = linearizaiton(g_dma_vision_receice_buff[3],0,45,0,1);
	}
	else if(g_dma_vision_receice_buff[0] == 0X5A && g_dma_vision_receice_buff[1] == 0X03)
	{
		wind_mode->mode = WIND_CYCLE;
		wind_mode->cycle_r = linearizaiton(g_dma_vision_receice_buff[2],15,35,8,10);
	}
	else if(g_dma_vision_receice_buff[0] == 0X5A && g_dma_vision_receice_buff[1] == 0X04)
	{
		wind_mode->mode = WIND_STOP;
		if(g_dma_vision_receice_buff[2] == 0X00) wind_mode->mode = WIND_CLOSE;
	}
}

static void set_wind_target(WindControl_t *target)
{
	target->sin_count ++;
	//风力摆位置设置
	if(target->mode == WIND_LINE)//直线模式
	{
		target->wind_pid_xangle.target = target->line_length*sin(2*3.1415926*(target->sin_count)/SIN_PERIOD);
		target->wind_pid_yangle.target = target->line_angle * target->line_length*sin(2*3.1415926*(target->sin_count)/SIN_PERIOD);
	}
	else if(target->mode == WIND_CYCLE)//画圆模式
	{
		target->wind_pid_xangle.target = target->cycle_r*cos(2*3.1415926*(target->sin_count)/SIN_PERIOD);
		target->wind_pid_yangle.target = -target->cycle_r*sin(2*3.1415926*(target->sin_count)/SIN_PERIOD);
	}
	else if(target->mode == WIND_STOP)
	{
		target->wind_pid_xangle.target = 0; 
		target->wind_pid_yangle.target = 0;
	}

	//对电机角度的限制
	amplitude_limit(&target->wind_pid_xangle.target, WIND_ANGLE_LIMIT);
	amplitude_limit(&target->wind_pid_yangle.target, WIND_ANGLE_LIMIT);
	if(target->sin_count == SIN_PERIOD) target->sin_count =0;
}

static void task_control_update(WindControl_t *update)
{
	update->wind_pid_xangle.measure = (float)stcAngle.Angle[0]/32768*180 - update->offset_xangle;
	update->wind_pid_yangle.measure = (float)stcAngle.Angle[1]/32768*180 - update->offset_yangle;
	
	update->wind_pid_xspeed.measure = (float)stcGyro.w[0]/32768*2000;
	update->wind_pid_yspeed.measure = (float)stcGyro.w[1]/32768*2000;
}

static void control_pid_calculate(WindControl_t *calculate)
{
	//pid计算
	pid_calculate( &(calculate->wind_pid_xspeed), &(calculate->wind_pid_xangle));
	pid_calculate( &(calculate->wind_pid_yspeed), &(calculate->wind_pid_yangle));
}

static float linearizaiton(float in , float xmin, float xmax, float ymin, float ymax)
{
	float a,b;
	float out;
	a = (ymax-ymin)/(xmax-xmin);
	b = ymax - a * xmax;
	out = a * in + b ;
	return out;
}
