#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H
#include <stm32f4xx.h>
#include <driver_control.h>
#include <driver_encoder.h>
/*�������*/
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

/*С���˶�����*/
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



#define BALL_MOTOR_LIMIT 30	//�Ե���Ƕȵ�����

typedef struct __ball_control_t
{
	const MotoMeasure_t *bull_xmotor_measure;
	const MotoMeasure_t *bull_ymotor_measure;
	pid_t motor_pid_xspeed;//����ٶȻ�
	pid_t motor_pid_yspeed;
	
	pid_t motor_pid_xangle;//����ǶȻ�
	pid_t motor_pid_yangle;
	
	pid_t bull_pid_xspeed;  //����ٶȻ�
	pid_t bull_pid_xangle;  //��ĽǶȻ�
	
	pid_t bull_pid_yspeed;  //����ٶȻ�
	pid_t bull_pid_yangle;  //��ĽǶȻ�
	
	int16_t xmotor_give_current; //�������ֵ
	int16_t ymotor_give_current; //�������ֵ
}BallControl_t;	

void control_task(void *pvParameters);
static void control_task_init(void);

static void set_motor_target(BallControl_t *target);
static void task_data_update(BallControl_t *update);
static void control_pid_calculate(BallControl_t *calculate);
static void set_motor_current(int16_t moto1, int16_t moto2);
#endif

