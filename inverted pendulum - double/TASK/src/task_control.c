#include <task_control.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f4xx_it.h>
#include <user_math.h>
#include <math.h>

float g_kp = 100.0f, g_ki=0.0f,g_kd=0.0f;
float g_kp_m = 0.0f, g_ki_m=0.0f,g_kd_m=0.0f;

float g_target_speed= 0.0f,g_target_angle=250.0f;
float g_maxout = 30000.0f;
float g_positon_maxout = 10000.0f;
float g_area = 5.0f;
float g_period = 950.0f;


InvertControl_t g_invert_control;

void control_task(void *pvParameters)
{
	vTaskDelay(1000);
	control_task_init();
	vTaskDelay(2000);
	while(1)
	{
		g_invert_control.circle_period = g_period;
		//电机目标值更新
		set_motor_target(&g_invert_control);
		//数据更新
		task_data_update(&g_invert_control);
		//pid控制更新
//		control_pid_calculate(&g_invert_control);
		//赋电流值
		set_motor_current(g_invert_control.motor_give_current);
		
		
		
		pid_reset(&g_invert_control.motor_pid_angle, g_kp_m,g_ki_m,g_kd_m);
		pid_reset(&g_invert_control.invert_pid_angle, g_kp,g_ki,g_kd);
		g_invert_control.invert_pid_angle.MaxOutput = g_maxout;
		g_invert_control.motor_pid_angle.MaxOutput = g_positon_maxout;
		g_invert_control.control_count++;
		vTaskDelay(TASK_CONTROL_PERIOD);
	}
}

static void control_task_init(void)
{
///*倒立摆速度环*/pid_struct_init(&g_invert_control.invert_pid_speed,POSITION_PID,ANGLE_LOOP,ECD_SPEED_MAXOUT, ECD_SPEED_INTEGRATION_LIMIT,
//					ECD_SPEED_KP,ECD_SPEED_KI,ECD_SPEED_KD);
/*倒立摆角度环*/pid_struct_init(&g_invert_control.invert_pid_angle,POSITION_PID,ANGLE_LOOP,ECD_ANGLE_MAXOUT, ECD_ANGLE_INTEGRATION_LIMIT,
					ECD_ANGLE_KP,ECD_ANGLE_KI,ECD_ANGLE_KD);
	
/*电机速度环*/pid_struct_init(&g_invert_control.motor_pid_speed,POSITION_PID,ANGLE_LOOP,MOTOR_SPEED_MAXOUT, MOTOR_SPEED_INTEGRATION_LIMIT,
					MOTOR_SPEED_KP,MOTOR_SPEED_KI,MOTOR_SPEED_KD);
/*电机角度环*/pid_struct_init(&g_invert_control.motor_pid_angle,POSITION_PID,ANGLE_LOOP,MOTOR_ANGLE_MAXOUT, MOTOR_ANGLE_INTEGRATION_LIMIT,
					MOTOR_ANGLE_KP,MOTOR_ANGLE_KI,MOTOR_ANGLE_KD);
	
	//获取电机测量指针
	g_invert_control.motor_measure = get_motor_measure_point();
	//设置电机的初始角度
	g_invert_control.motor_pid_angle.target = 220.0f;
	//得到编码器的初始角度
	g_invert_control.g_encoder_data.ecd = TIM_GetCounter(TIM4);
	g_invert_control.g_encoder_data.current_angle = (g_invert_control.g_encoder_data.ecd + (g_invert_control.g_encoder_data.round_cnt - 1) * 8000.0f) * ENCODER_ECD_TO_DEG ;
	//电机的初始角度
	g_invert_control.current_angle = g_invert_control.motor_measure->current_angle;
	//设置运动周期
	g_invert_control.swing_period = 850;
	g_invert_control.circle_period = 900;
	
	g_invert_control.mode = ABNORMAL_MODE;
	
}

static void set_motor_target(InvertControl_t *target)
{
	target->sin_count ++;
	target->count ++;
	/*根据模式对电机目标值进行选择*/
	if(target->mode == SWING_MODE)/*摆动*/
	{
		target->motor_pid_angle.target = 220.0f + 15 * sin(2*3.1415926*(target->sin_count)/g_invert_control.swing_period);
		//把sin函数计数值改变
		if(target->sin_count == g_invert_control.swing_period) target->sin_count =0;
	}
	
	else if(target->mode == CIRCLE_MODE)/*圆周运动*/
	{
		pid_reset(&g_invert_control.motor_pid_angle, 50.0f,0.0f,0.0f);
		if(target->circle_flag == 0) //没有达到180度
		{
			target->motor_pid_angle.target = 220.0f + 50 * sin(2*3.1415926*(target->sin_count)/g_invert_control.circle_period);
			if( (ABS(target->g_encoder_data.current_angle - 180.0f) < 1.0f))//没有达到180度时
			{
				target->circle_flag = 1;
				g_invert_control.circle_period = 350.0f;
			}
		}
		else //达到180度后直接做圆周运动
		{
			target->motor_pid_angle.target = 220.0f + 5 * sin(2*3.1415926*(target->sin_count)/g_invert_control.circle_period);
		}
		//把sin函数计数值改变
		if(target->sin_count == g_invert_control.circle_period) target->sin_count =0;
	}
	
	
	/*非自然倒立模式*/
	else if(target->mode == ABNORMAL_MODE)
	{
		//设置倒立摆的目标值
		target->invert_pid_angle.target = 180.0f;//立起来中间的角度
		target->motor_pid_angle.target = 220.f;//设置电机定摆时的角度
		//倒立摆角度进行计算
		single_pid_calculate(&target->invert_pid_angle);
		if(target ->control_count == 5)
		{
			pid_calculate(&target->motor_pid_speed, &target->motor_pid_angle);
			target->control_count = 0;
		}
		//当与180度夹角小于15度时，开始倒立
		if(ABS( ABS(target->g_encoder_data.current_angle) - 180.0f) < 15.0f)
		{
			target->motor_give_current = target->invert_pid_angle.pos_out  + target->motor_pid_angle.pos_out;
		}
		else 
		{
			target->motor_give_current = 0;
		}
	}
	

	else if(target->mode == NORMAL_MODE)/*自然倒立模式*/
	{
		if(target->normal_flag == 0) //没有达到180度
		{
			
			
			pid_reset(&g_invert_control.motor_pid_angle, 3.0f,0.0f,0.0f);
//			target->motor_pid_angle.target = 220.0f + 30 * sin(2*3.1415926*(target->sin_count)/g_invert_control.circle_period);
			
			switch(target->count)
			{
				case 1:
					pid_reset(&g_invert_control.motor_pid_angle, 100.0f,0.0f,0.0f);
					target->motor_pid_angle.target = 160.0f;
					break;
				case 200:
					pid_reset(&g_invert_control.motor_pid_angle, 3.0f,0.0f,0.0f);
					target->motor_pid_angle.target = 220.0f;
					break;
			}
			if(target->count == 800) target->count = 0;
			if( (ABS( ABS(target->g_encoder_data.current_angle) - 180.0f) < 15.0f))//没有达到180度时
			{
				target->normal_flag = 1;
			}
		}
		else //达到180度后进行倒立模式
		{
			pid_reset(&g_invert_control.motor_pid_angle, 3.0f,0.0f,0.0f);
			
			//设置倒立摆的目标值
			target->invert_pid_angle.target = 180.0f;//立起来中间的角度
			//倒立摆角度进行计算
			pid_calculate(&target->invert_pid_speed, &target->invert_pid_angle);
			target->motor_pid_angle.target = 220.0f + target->invert_pid_angle.pos_out;
		}
		//把sin函数计数值改变
		if(target->sin_count == g_invert_control.circle_period) target->sin_count =0;
	}
	
}

static void task_data_update(InvertControl_t *update)
{
	//编码器角度更新
	update->g_encoder_data.ecd = TIM_GetCounter(TIM4);
	update->g_encoder_data.current_angle = (update->g_encoder_data.ecd + (update->g_encoder_data.round_cnt - 1) * 8000.0f) * ENCODER_ECD_TO_DEG;
	//编码器角度传递
	update->invert_pid_angle.measure = update->g_encoder_data.current_angle;
	//电机数据更新
	update->last_angle = update->current_angle;
	update->current_angle = update->motor_measure->current_angle;
	
	if( (update->current_angle - update->last_angle) < -180.0f)
	{
		update->round_cnt ++;
	}	
	else if( (update->current_angle - update->last_angle) > 180.0f)
	{
		update->round_cnt --;
	}
	
	update->motor_pid_speed.measure = update->motor_measure->speed_rpm;
	update->motor_pid_angle.measure = update->current_angle + update->round_cnt *360.0f;
}

static void control_pid_calculate(InvertControl_t *calculate)
{
	//pid计算
	pid_calculate( &(calculate->motor_pid_speed), &(calculate->motor_pid_angle));

	//电机电流值赋值
	calculate->motor_give_current = calculate->motor_pid_speed.pos_out;
}


static void set_motor_current(int16_t moto1)
{
	CanTxMsg CAN1_CouldMotorStr;
	
	CAN1_CouldMotorStr.StdId=0x1FF;
	CAN1_CouldMotorStr.IDE=CAN_Id_Standard;
	CAN1_CouldMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_CouldMotorStr.DLC=0x08;
	CAN1_CouldMotorStr.Data[0]= (moto1 >> 8);
	CAN1_CouldMotorStr.Data[1]= moto1;
	CAN1_CouldMotorStr.Data[2]= 0;
	CAN1_CouldMotorStr.Data[3]= 0;
	CAN1_CouldMotorStr.Data[4]= 0;
	CAN1_CouldMotorStr.Data[5]= 0;
	CAN1_CouldMotorStr.Data[6]= 0;
	CAN1_CouldMotorStr.Data[7]= 0;
	
	CAN_Transmit(CAN1,&CAN1_CouldMotorStr);
}
