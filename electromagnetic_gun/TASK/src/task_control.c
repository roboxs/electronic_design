#include <FreeRTOS.h>
#include <task.h>
#include <task_control.h>
#include <task_led.h>
#include <driver_control.h>
#include <stm32f4xx_it.h>
#include <task_vision.h>
#include <user_math.h>
#include <bsp_gpio.h>


float yaw_target = -2.0f,pitch_target = 121.48f;
float g_kp = 80.0f,g_ki = 0.1f,g_kd = 0.0f;
float g_IntegralLimit = 1000.0f;

float g_kp_angle = 3.5f,g_ki_angle = 0.0f,g_kd_angle= 0.0f;
float g_angle_IntegralLimit = 100.0f;

float g_d = 250.0f;
uint8_t g_mode = 0;
float g_alpha = 10.0f;

Gimbal_t g_gimbal_control;
RampStr_t scan_ramp;

void control_task(void *pvParameters)
{
	//等待电调启动
	vTaskDelay(1500);
	task_init();
	while(1)
	{
		//设置电机的目标值
		set_system_target(&g_gimbal_control);
		//更新当前测量值
		update_measure(&g_gimbal_control);
		//计算任务
		task_calculate(&g_gimbal_control);
		//赋电流值
		set_motor_current(g_gimbal_control.yaw_current, g_gimbal_control.pitch_current);
//		pid_reset(&g_gimbal_control.pitch_pid_speed,g_kp,g_ki,g_kd);
//		pid_reset(&g_gimbal_control.yaw_pid_angle,g_kp_angle,g_ki_angle,g_kd_angle);
		/*执行射击命令*/
		if(g_gimbal_control.cmd == CMD_SHOOT && g_gimbal_control.last_cmd != CMD_SHOOT) 
		{
			g_gimbal_control.shoot_flag = 1;//产生射击标志位
		}
		/*射击标志位*/
		if(g_gimbal_control.shoot_flag == 1)
		{
//			if(g_gimbal_control.mode == ADVANCED_MODE1)	g_gimbal_control.shoot_success_flag = 1;
			if(g_gimbal_control.out_flag != 1) //充电时不能放电
			{
				g_gimbal_control.batter_time++;
				POWER_BARRERY_CMD = 1;//开始充电
				LED6 = 0;
				if(g_gimbal_control.batter_time == BATTERY_TIME)
				{
					//充电完成
					LED6 = 1;
					POWER_BARRERY_CMD = 0;
					g_gimbal_control.out_flag = 1;//此时可以放电
					g_gimbal_control.batter_time = 0;
				}
			}
			else if(g_gimbal_control.out_flag == 1)
			{
				g_gimbal_control.shoot_time ++;
				if(g_gimbal_control.shoot_time == SHOOT_TIME)//2s
				{
					LED8 = 0;
					GPIO_SetBits(GPIOD,GPIO_Pin_12);

				}					
				else if(g_gimbal_control.shoot_time == SHOOT_CLOSE_TIME)
				{
					LED8 = 1;
					GPIO_ResetBits(GPIOD,GPIO_Pin_12);				
					g_gimbal_control.shoot_flag = 0;
					g_gimbal_control.shoot_time = 0;
					g_gimbal_control.out_flag = 0;
				}
			}
		}
		LED3=!LED3;
		vTaskDelay(TASK_CONTROL_CYCLE);
	}
}

static void task_init(void)
{
/*yaw速度环*/pid_struct_init(&g_gimbal_control.yaw_pid_speed,POSITION_PID,DOUBLE_LOOP,YAW_SPEED_MAXOUT, YAW_SPEED_INTEGRATION_LIMIT,
					YAW_SPEED_KP,YAW_SPEED_KI,YAW_SPEED_KD);
/*yaw角度环*/pid_struct_init(&g_gimbal_control.yaw_pid_angle,POSITION_PID,DOUBLE_LOOP,YAW_ANGLE_MAXOUT, YAW_ANGLE_INTEGRATION_LIMIT,
					YAW_ANGLE_KP,YAW_ANGLE_KI,YAW_ANGLE_KD);
	
/*yaw速度环*/pid_struct_init(&g_gimbal_control.pitch_pid_speed,POSITION_PID,DOUBLE_LOOP,PITCH_SPEED_MAXOUT, PITCH_SPEED_INTEGRATION_LIMIT,
					PITCH_SPEED_KP,PITCH_SPEED_KI,PITCH_SPEED_KD);
/*yaw角度环*/pid_struct_init(&g_gimbal_control.pitch_pid_angle,POSITION_PID,DOUBLE_LOOP,PITCH_ANGLE_MAXOUT, PITCH_ANGLE_INTEGRATION_LIMIT,
					PITCH_ANGLE_KP,PITCH_ANGLE_KI,PITCH_ANGLE_KD);
/*视觉误差环*/pid_struct_init(&g_gimbal_control.vision_pid_angle,POSITION_PID,ANGLE_LOOP,VISION_ANGLE_MAXOUT, VISION_ANGLE_INTEGRATION_LIMIT,
					VISION_ANGLE_KP,VISION_ANGLE_KI,VISION_ANGLE_KD);
	
	g_gimbal_control.yaw_pid_angle.alpha = 0.5;
	g_gimbal_control.pitch_pid_angle.alpha = 0.5;
	//yaw轴测量值传递 
	g_gimbal_control.yaw_measure =  get_moto6020_yaw_point();
	//pitch轴测量值传递
	g_gimbal_control.pitch_measure = get_moto6020_pitch_point();
	
	//设置电机的初始角度
	g_gimbal_control.yaw_pid_angle.target = YAW_INIT_ANGLE;
	g_gimbal_control.pitch_pid_angle.target = PITCH_INIT_ANGLE;
	
//	g_gimbal_control.shoot_flag = 1;
}


float pitch_add,yaw_add;
static void set_system_target(Gimbal_t *target)
{
	target->last_mode = target->mode;//上次的模式
	target->mode = g_bluetooth_data.mode;
//	target->mode = ADVANCED_MODE2;
	
//	target->yaw_pid_angle.target = yaw_target;
//	target->pitch_pid_angle.target = pitch_target;
	/*两次模式不同时，才执行模式中的内容*/
	if(target->mode == BASIS_MODE1 && target->last_mode != BASIS_MODE1)
	{
		target->pitch_pid_angle.target =PITCH_INIT_ANGLE;
		target->yaw_pid_angle.target = YAW_INIT_ANGLE ;
	}
	else if(target->mode == BASIS_MODE2 && target->last_mode != BASIS_MODE2)/*输入d 抬升枪口打击*/
	{
		target->target_d = g_bluetooth_data.d;
		pitch_add = Triangle_fitting(target->target_d);
//		pitch_add = target->target_d;
		yaw_add = 0.0f;
		target->pitch_pid_angle.target -=yaw_add;
		target->yaw_pid_angle.target -= pitch_add;
		target->shoot_flag = 1;
	}
	else if(target->mode == BASIS_MODE3 && target->last_mode != BASIS_MODE3)/*输入d以及α射击*/
	{
		target->alpha = g_bluetooth_data.alpha;
		target->target_d = g_bluetooth_data.d;
		pitch_add = Triangle_fitting(target->target_d);
//		pitch_add = target->target_d;
		yaw_add = target->alpha;
		target->pitch_pid_angle.target -=yaw_add;
		target->yaw_pid_angle.target -= pitch_add;
		target->shoot_flag = 1;
	}
	else if(target->mode == ADVANCED_MODE1)/*一键启动自动射击*/
	{
		//采用视觉模式
		target->pitch_pid_angle.target = g_vision_data.get_target_angle_yaw;
		target->yaw_pid_angle.target = YAW_INIT_ANGLE - Triangle_fitting(target->measure_d);
		//此时判断可以射击
		if(ABS(g_gimbal_control.yaw_pid_angle.target - g_gimbal_control.yaw_pid_angle.measure) < 0.5f && ABS(g_gimbal_control.pitch_pid_angle.target - g_gimbal_control.pitch_pid_angle.measure) < 0.5f)
		{
			 target->shoot_flag = 1;
		}
	}
	else if(target->mode == ADVANCED_MODE2)/*左右摇头，自动射击*/
	{
		//首先yaw正弦旋转，开始搜索
		target->sin_count++;
		if(g_gimbal_control.out_flag != 1) //充电时不能放电
		{
			g_gimbal_control.batter_time++;
			POWER_BARRERY_CMD = 1;//开始充电
			LED6 = 0;
			if(g_gimbal_control.batter_time == BATTERY_TIME)
			{
				//充电完成
				LED6 = 1;
				POWER_BARRERY_CMD = 0;
				g_gimbal_control.out_flag = 1;//此时可以放电
				g_gimbal_control.batter_time = 0;
			}
		}
		
		if(target ->ramp_flag == 0)
		{
			target->yaw_pid_angle.target = -30 + 60 * 0.0004f * target->sin_count;//起初为最右边
		}
		else if(target ->ramp_flag == 1)
		{
			target->yaw_pid_angle.target =  30 - 60 * 0.0004f * target->sin_count;//判断到最左边
		}		
		
		if(ABS(target->yaw_pid_angle.target - 30) < 0.01f)//判断到最左边
		{
			target ->ramp_flag = 1;
		}
		else if(ABS(target->yaw_pid_angle.target + 30) < 0.01f)
		{
			target ->ramp_flag = 0;
		}
		target->pitch_pid_angle.target = PITCH_INIT_ANGLE - 21.5f;
		if(ABS(target->yaw_pid_angle.target - g_vision_data.get_target_angle_yaw) < 1.0f)//判断和视觉角度的重合程度
		{
			if(g_gimbal_control.out_flag == 1)
			{
				g_gimbal_control.shoot_time ++;
				if(g_gimbal_control.shoot_time == 200)//2s
				{
					LED8 = 0;
					GPIO_SetBits(GPIOD,GPIO_Pin_12);
				}					
				else if(g_gimbal_control.shoot_time == 800)
				{
					LED8 = 1;
					GPIO_ResetBits(GPIOD,GPIO_Pin_12);				
					g_gimbal_control.shoot_flag = 0;
					g_gimbal_control.shoot_time = 0;
					g_gimbal_control.out_flag = 0;
				}
			}
		}
		if(target->sin_count == SIN_PERIOD) target->sin_count = 0;
	}
	
}


static void update_measure(Gimbal_t *update)
{
	//yaw测量值更新
	update->yaw_pid_speed.measure = update->yaw_measure->speed_rpm;
	update->yaw_pid_angle.measure = update->yaw_measure->total_angle -112.0f;
	//pitch测量值更新
	update->pitch_pid_speed.measure = update->pitch_measure->speed_rpm;
	update->pitch_pid_angle.measure = update->pitch_measure->total_angle;
	//更新靶子当前的距离
	update->measure_d = g_vision_data.get_current_distance / 10;
	//更新当前的命令
	update->last_cmd = update->cmd;
	update->cmd = g_bluetooth_data.cmd;
	//视觉角度更新
	update->vision_pid_angle.measure = g_vision_data.get_target_angle_yaw;
//	update->cmd = 1;//发送射击命令
}

static void task_calculate(Gimbal_t * cal)
{
	single_pid_calculate(&cal->yaw_pid_angle);
	single_pid_calculate(&cal->pitch_pid_angle);

	cal->yaw_current = cal->yaw_pid_angle.pos_out;//计算电流值
	cal->pitch_current = cal->pitch_pid_angle.pos_out;//计算电流值
}
static void set_motor_current(int16_t moto1, int16_t moto2)
{
	CanTxMsg CAN1_CouldMotorStr;
	
	CAN1_CouldMotorStr.StdId=0X1FF;
	CAN1_CouldMotorStr.IDE=CAN_Id_Standard;
	CAN1_CouldMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_CouldMotorStr.DLC=0x08;
	CAN1_CouldMotorStr.Data[0]= (moto1 >> 8);
	CAN1_CouldMotorStr.Data[1]= moto1;
	CAN1_CouldMotorStr.Data[2]= (moto2 >> 8);
	CAN1_CouldMotorStr.Data[3]= moto2;
	CAN1_CouldMotorStr.Data[4]= 0;
	CAN1_CouldMotorStr.Data[5]= 0;
	CAN1_CouldMotorStr.Data[6]= 0;
	CAN1_CouldMotorStr.Data[7]= 0;
	
	CAN_Transmit(CAN1,&CAN1_CouldMotorStr);
}
// 
//static float linearizaiton(float in , float xmin, float xmax, float ymin, float ymax)
//{
//	float a,b;
//	float out;
//	a = (ymax-ymin)/(xmax-xmin);
//	b = ymax - a * xmax;
//	out = a * in + b ;
////	if(out < ymin) out = ymin;
////	else if(out > ymax) out = ymax;
//	return out;
//}

static float quadratic_fitting(float distance)
{
	float temp;
	temp = -0.1571f * distance *distance + 13.0536f * distance + 30.4733f;
	return temp;
}


static float three_fitting(float distance)
{
	float temp;
	temp = -1.5076e-5f * distance *distance*distance + 0.0125f * distance * distance - 3.1982f * distance + 278.1027;
	return temp;
}

static float Triangle_fitting(float distance)
{
	float temp;
//	temp = 28.99f + 5.354f * cos(0.02121f * distance) + 10.49f * sin(0.02121f*distance);
	temp = 149.8f - 99.71f * cos(0.004164f * distance) - 89.97f * sin(0.004164f*distance);
	return temp;
}