#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H
#include <stm32f4xx.h>
#include <driver_control.h>
#include <driver_encoder.h>
/*电机参数*/
#define MOTOR_BALL_XANGLE_KP 100
#define MOTOR_BALL_XANGLE_KI 0
#define MOTOR_BALL_XANGLE_KD 0
#define MOTOR_BALL_XANGLE_MAXOUT 3000
#define MOTOR_BALL_XANGLE_INTEGRATION_LIMIT 1000

#define MOTOR_BALL_YANGLE_KP 100
#define MOTOR_BALL_YANGLE_KI 0
#define MOTOR_BALL_YANGLE_KD 0
#define MOTOR_BALL_YANGLE_MAXOUT 3000
#define MOTOR_BALL_YANGLE_INTEGRATION_LIMIT 1000

#define MOTOR_BALL_SPEED_KP 5
#define MOTOR_BALL_SPEED_KI 0.1
#define MOTOR_BALL_SPEED_KD 0
#define MOTOR_BALL_SPEED_MAXOUT 15000
#define MOTOR_BALL_SPEED_INTEGRATION_LIMIT 5000

/*小球运动参数*/
#define BALL_XANGLE_KP -0.1
#define BALL_XANGLE_KI 0
#define BALL_XANGLE_KD 0
#define BALL_XANGLE_MAXOUT 30
#define BALL_XANGLE_INTEGRATION_LIMIT 10

#define BALL_YANGLE_KP 0.1
#define BALL_YANGLE_KI 0
#define BALL_YANGLE_KD 0
#define BALL_YANGLE_MAXOUT 30
#define BALL_YANGLE_INTEGRATION_LIMIT 10

#define BALL_SPEED_KP 5
#define BALL_SPEED_KI 0.1
#define BALL_SPEED_KD 0
#define BALL_SPEED_MAXOUT 30
#define BALL_SPEED_INTEGRATION_LIMIT 0



#define BALL_MOTOR_LIMIT 30	//对电机角度的限制

typedef struct __ball_control_t
{
	const MotoMeasure_t *bull_xmotor_measure;
	const MotoMeasure_t *bull_ymotor_measure;
	pid_t motor_pid_xspeed;//电机速度环
	pid_t motor_pid_yspeed;
	
	pid_t motor_pid_xangle;//电机角度环
	pid_t motor_pid_yangle;
	
	pid_t bull_pid_xspeed;  //球的速度环
	pid_t bull_pid_xangle;  //球的角度环
	
	pid_t bull_pid_yspeed;  //球的速度环
	pid_t bull_pid_yangle;  //球的角度环
	
	int16_t xmotor_give_current; //电机电流值
	int16_t ymotor_give_current; //电机电流值
}BallControl_t;	

void control_task(void *pvParameters);
static void control_task_init(void);

static void set_motor_target(BallControl_t *target);
static void task_data_update(BallControl_t *update);
static void control_pid_calculate(BallControl_t *calculate);
static void set_motor_current(int16_t moto1, int16_t moto2);
#endif

