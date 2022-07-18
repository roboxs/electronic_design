#include <task_control.h>
#include <FreeRTOS.h>
#include <task.h>
#include <task_led.h>
#include <driver_control.h>
#include <stm32f4xx_it.h>
#include <user_math.h>
#include <task_vision.h>


BallControl_t g_ball_move;
float g_kp_x=4.0f,g_ki_x=0.0f,g_kd_x=20.0f;
float g_kp_ax=0.04f,g_ki_ax=0.0f,g_kd_ax=0.0f;
float g_anti_x = 0.0f;
float g_IntegralLimit_x = 0.0f;
float g_maxout_x= 38.0f;

float g_kp_y=-4.0f,g_ki_y=-0.0f,g_kd_y=-20.0f;
float g_kp_ay=0.037f,g_ki_ay=0.0f,g_kd_ay=0.0f;
float g_anti_y = 0.0f;
float g_IntegralLimit_y = 0.0f;
float g_maxout_y = 38.0f;

float g_target_x = -100.0f,g_target_y = 0.0f;
float g_deadband_y = 0.0f;
float g_deadband_x = 0.0f;
float g_kp_m=100.0f,g_ki_m=0.0f,g_kd_m=0.0f;
float g_alpha_y =0.7f;
float g_alpha_x =0.7f;

void control_task(void *pvParameters)
{
	vTaskDelay(3000);//等待电调启动
	control_task_init();
	while(1)
	{
		//电机目标值更新
		set_motor_target(&g_ball_move);
		//任务数据更新
		task_data_update(&g_ball_move);
		//pid控制更新
		control_pid_calculate(&g_ball_move);
		//给can赋值
		set_motor_current(g_ball_move.xmotor_give_current,g_ball_move.ymotor_give_current);
//		set_motor_current(0,g_ball_move.ymotor_give_current);
//		set_motor_current(g_ball_move.xmotor_give_current,0);
		pid_reset(&g_ball_move.bull_pid_xspeed,g_kp_x,g_ki_x,g_kd_x);
		pid_reset(&g_ball_move.bull_pid_xangle,g_kp_ax,g_ki_ax,g_kd_ax);
		
		pid_reset(&g_ball_move.bull_pid_yspeed,g_kp_y,g_ki_y,g_kd_y);
		pid_reset(&g_ball_move.bull_pid_yangle,g_kp_ay,g_ki_ay,g_kd_ay);
		
		pid_reset(&g_ball_move.motor_pid_yangle,g_kp_m,g_ki_m,g_kd_m);
		
		g_ball_move.bull_pid_xangle.integral.err_min = g_anti_x;
		g_ball_move.bull_pid_yangle.integral.err_min = g_anti_y;
		g_ball_move.bull_pid_xangle.IntegralLimit = g_IntegralLimit_x;
		g_ball_move.bull_pid_yangle.IntegralLimit = g_IntegralLimit_y;
//		g_ball_move.bull_pid_xangle.MaxOutput = g_maxout_x;
//		g_ball_move.bull_pid_yangle.MaxOutput = g_maxout_y;
		
		g_ball_move.bull_pid_xangle.deadband = g_deadband_x;
		g_ball_move.bull_pid_yangle.deadband = g_deadband_y;
//		g_ball_move.bull_pid_yangle.alpha = g_alpha_y;
//		g_ball_move.bull_pid_xangle.alpha = g_alpha_x;
		LED5=!LED5;
		vTaskDelay(10);
	}
}


static void control_task_init(void)
{
//电机双环控制
/*x方向电机速度环*/	pid_struct_init(&g_ball_move.motor_pid_xspeed,POSITION_PID,DOUBLE_LOOP,MOTOR_BALL_SPEED_MAXOUT, MOTOR_BALL_SPEED_INTEGRATION_LIMIT,
					MOTOR_BALL_SPEED_KP,MOTOR_BALL_SPEED_KI,MOTOR_BALL_SPEED_KD);
/*x方向电机角度环*/	pid_struct_init(&g_ball_move.motor_pid_xangle,POSITION_PID,DOUBLE_LOOP,MOTOR_BALL_XANGLE_MAXOUT,MOTOR_BALL_XANGLE_INTEGRATION_LIMIT,
					MOTOR_BALL_XANGLE_KP,MOTOR_BALL_XANGLE_KI,MOTOR_BALL_XANGLE_KD);
/*y方向电机速度环*/	pid_struct_init(&g_ball_move.motor_pid_yspeed,POSITION_PID,DOUBLE_LOOP,MOTOR_BALL_SPEED_MAXOUT, MOTOR_BALL_SPEED_INTEGRATION_LIMIT,
					MOTOR_BALL_SPEED_KP,MOTOR_BALL_SPEED_KI,MOTOR_BALL_SPEED_KD);
/*y方向电机角度环*/	pid_struct_init(&g_ball_move.motor_pid_yangle,POSITION_PID,DOUBLE_LOOP,MOTOR_BALL_YANGLE_MAXOUT,MOTOR_BALL_YANGLE_INTEGRATION_LIMIT,
					MOTOR_BALL_YANGLE_KP,MOTOR_BALL_YANGLE_KI,MOTOR_BALL_YANGLE_KD);
//小球单环位置控制
/*小球x轴速度环*/pid_struct_init(&g_ball_move.bull_pid_xspeed,POSITION_PID,DOUBLE_LOOP,BALL_SPEED_MAXOUT, BALL_SPEED_INTEGRATION_LIMIT,
					BALL_SPEED_KP,BALL_SPEED_KI,BALL_SPEED_KD);
/*小球x轴角度环*/pid_struct_init(&g_ball_move.bull_pid_xangle,POSITION_PID,DOUBLE_LOOP,BALL_XANGLE_MAXOUT, BALL_XANGLE_INTEGRATION_LIMIT,
					BALL_XANGLE_KP,BALL_XANGLE_KI,BALL_XANGLE_KD);
/*小球y轴速度环*/pid_struct_init(&g_ball_move.bull_pid_yspeed,POSITION_PID,DOUBLE_LOOP,BALL_SPEED_MAXOUT, BALL_SPEED_INTEGRATION_LIMIT,
					BALL_SPEED_KP,BALL_SPEED_KI,BALL_SPEED_KD);
/*小球y轴角度环*/pid_struct_init(&g_ball_move.bull_pid_yangle,POSITION_PID,DOUBLE_LOOP,BALL_YANGLE_MAXOUT, BALL_YANGLE_INTEGRATION_LIMIT,
					BALL_YANGLE_KP,BALL_YANGLE_KI,BALL_YANGLE_KD);
	//设置电机的死区
	g_ball_move.motor_pid_xangle.deadband = 0.3;
	g_ball_move.motor_pid_yangle.deadband = 0.3;
	
	//获取电机反馈值变量的地址
	g_ball_move.bull_xmotor_measure = get_bull_xmotor_measure_point();
	g_ball_move.bull_ymotor_measure = get_bull_ymotor_measure_point();
}
static void set_motor_target(BallControl_t *target)
{
	//原点位置
	target->bull_pid_xangle.target = g_target_x;
	target->bull_pid_yangle.target = g_target_y;
	//对电机目标角度的跟随设定 利用误差控制
	pid_calculate(&target->bull_pid_xspeed, &target->bull_pid_xangle);
	pid_calculate(&target->bull_pid_yspeed, &target->bull_pid_yangle);
//	single_pid_calculate(&target->bull_pid_xangle);
//	single_pid_calculate(&target->bull_pid_yangle);
	
	target->motor_pid_xangle.target = target->bull_pid_xspeed.pos_out;
	target->motor_pid_yangle.target = target->bull_pid_yspeed.pos_out;
	//对电机角度的限制
	amplitude_limit(&target->motor_pid_xangle.target, BALL_MOTOR_LIMIT);
	amplitude_limit(&target->motor_pid_yangle.target, BALL_MOTOR_LIMIT);
}


static void task_data_update(BallControl_t *update)
{
	//x轴电机速度
	update->motor_pid_xspeed.measure = update->bull_xmotor_measure->speed_rpm;
	//x轴电机角度
	update->motor_pid_xangle.measure = (update->bull_xmotor_measure->round_cnt * 8191 + update->bull_xmotor_measure->ecd - update->bull_xmotor_measure->offset_ecd) * MOTOR_ECD_TO_DEG;
	//y轴电机速度
	update->motor_pid_yspeed.measure = update->bull_ymotor_measure->speed_rpm;
	//y轴电机角度
	update->motor_pid_yangle.measure = (update->bull_ymotor_measure->round_cnt * 8191 + update->bull_ymotor_measure->ecd - update->bull_ymotor_measure->offset_ecd) * MOTOR_ECD_TO_DEG;
	/*小球位置更新*/
	update->bull_pid_xangle.measure = g_vision_data.get_target_angle_x;
	update->bull_pid_yangle.measure = g_vision_data.get_target_angle_y;
	/*小球速度更新*/
	update->bull_pid_xspeed.measure = g_vision_data.after_xspeed;//位置差作为速度更新
	update->bull_pid_yspeed.measure = g_vision_data.after_yspeed;//位置差作为速度更新
}

static void control_pid_calculate(BallControl_t *calculate)
{
	//pid计算
	pid_calculate( &(calculate->motor_pid_xspeed), &(calculate->motor_pid_xangle));
	pid_calculate( &(calculate->motor_pid_yspeed), &(calculate->motor_pid_yangle));
	
	//电机电流值赋值
	calculate->xmotor_give_current = calculate->motor_pid_xspeed.pos_out;
	calculate->ymotor_give_current = calculate->motor_pid_yspeed.pos_out;
}

static void set_motor_current(int16_t moto1, int16_t moto2)
{
	CanTxMsg CAN1_CouldMotorStr;
	
	CAN1_CouldMotorStr.StdId=0X200;
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
