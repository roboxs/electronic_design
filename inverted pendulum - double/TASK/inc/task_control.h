#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

#include <stm32f4xx.h>
#include <driver_control.h>
#include <driver_encoder.h>

/*编码器的参数*/
#define ECD_SPEED_KP 50
#define ECD_SPEED_KI 0
#define ECD_SPEED_KD 0
#define ECD_SPEED_MAXOUT 3000
#define ECD_SPEED_INTEGRATION_LIMIT 1000

#define ECD_ANGLE_KP -18
#define ECD_ANGLE_KI 0
#define ECD_ANGLE_KD -100
#define ECD_ANGLE_MAXOUT 100
#define ECD_ANGLE_INTEGRATION_LIMIT 50

/*电机的参数*/
#define MOTOR_SPEED_KP 80
#define MOTOR_SPEED_KI 5
#define MOTOR_SPEED_KD 0
#define MOTOR_SPEED_MAXOUT 30000
#define MOTOR_SPEED_INTEGRATION_LIMIT 300

#define MOTOR_ANGLE_KP 3.0
#define MOTOR_ANGLE_KI 0
#define MOTOR_ANGLE_KD 0
#define MOTOR_ANGLE_MAXOUT 80
#define MOTOR_ANGLE_INTEGRATION_LIMIT 0


#define TASK_CONTROL_PERIOD 5		//控制任务周期
//#define SIN_PERIOD		180.0f//sin函数周期，和控制任务周期相关
#define SIN_PERIOD			180.0f//sin函数周期，和控制任务周期相关

typedef enum
{
	SWING_MODE=0,
	CIRCLE_MODE=1,
	ABNORMAL_MODE=2,//非自然倒立模式
	NORMAL_MODE=3,//自然倒立模式
}InvertMode_e;

typedef struct __invert_control_t
{
	uint8_t circle_flag;
	uint8_t normal_flag;
//	uint8_t 
	
	InvertMode_e mode;//倒立摆模式
	
	pid_t invert_pid_speed;//倒立摆自身的速度环
	pid_t invert_pid_angle;//倒立摆自身的角度环
	
	pid_t motor_pid_speed;//旋转电机的速度环
	pid_t motor_pid_angle;//旋转电机的角度环
	MotoMeasure_t g_encoder_data;//编码器角度测量
	const MotoMeasure_t *motor_measure;
	
	int16_t motor_give_current; //电机电流值	
	int circle_period;
	int swing_period;
	uint32_t sin_count;
	uint32_t count;
	uint32_t control_count;
	
	float last_angle;
	float current_angle;
	int32_t  round_cnt;
}InvertControl_t;	

void control_task(void *pvParameters);
static void control_task_init(void);
static void set_motor_target(InvertControl_t *target);
static void task_data_update(InvertControl_t *update);
static void control_pid_calculate(InvertControl_t *calculate);
static void set_motor_current(int16_t moto1);




extern InvertControl_t g_invert_control;
#endif

