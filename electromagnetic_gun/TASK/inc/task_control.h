#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H
#include <stm32f4xx.h>
#include <driver_control.h>
#include <driver_encoder.h>
#include <FreeRTOS.h>
#include <task.h>

#define DISTANCE_MIN 200		//靶子的距离
#define DISTANCE_MAX 300	
#define PITCH_UP_MIN 0			//YAW轴抬升的角度
#define PITCH_UP_MAX 30 

#define SHOOT_TIME			2000//每个子弹发射时间20ms
#define SHOOT_CLOSE_TIME 	3000
#define BATTERY_TIME		5000//充电时间
#define CLOSE_BATTERY_TIME 	5500

#define YAW_INIT_ANGLE 		0.0f
#define PITCH_INIT_ANGLE	50.0f


#define YAW_SCAN_MAX 		30.0f //

#define TASK_CONTROL_CYCLE 1
#define SIN_PERIOD			2500//yaw摇摆周期和控制任务周期有关

#define YAW_SPEED_KP 80
#define YAW_SPEED_KI 5
#define YAW_SPEED_KD 0
#define	YAW_SPEED_MAXOUT 20000
#define YAW_SPEED_INTEGRATION_LIMIT 10000
#define YAW_ANGLE_KP 150
#define YAW_ANGLE_KI 0
#define YAW_ANGLE_KD 3000
#define	YAW_ANGLE_MAXOUT 4800
#define YAW_ANGLE_INTEGRATION_LIMIT 0

#define PITCH_SPEED_KP 80
#define PITCH_SPEED_KI 3
#define PITCH_SPEED_KD 0
#define	PITCH_SPEED_MAXOUT 20000
#define PITCH_SPEED_INTEGRATION_LIMIT 10000
#define PITCH_ANGLE_KP 150
#define PITCH_ANGLE_KI 0
#define PITCH_ANGLE_KD 5000
#define	PITCH_ANGLE_MAXOUT 5000
#define PITCH_ANGLE_INTEGRATION_LIMIT 2000

#define VISION_ANGLE_KP 1
#define VISION_ANGLE_KI 0
#define VISION_ANGLE_KD 0
#define	VISION_ANGLE_MAXOUT 30
#define VISION_ANGLE_INTEGRATION_LIMIT 0

typedef enum __gimbal_mode_e
{
	READY_MODE =0,
	BASIS_MODE1=1,
	BASIS_MODE2=2,
	BASIS_MODE3=3,
	ADVANCED_MODE1=4,
	ADVANCED_MODE2=5,
	ADVANCED_MODE3=6,
}GimbalMode_e;

typedef enum __gimbal_cmd_e
{
	CMD_RUNNING=0,
	CMD_SHOOT=1,
	CMD_RESET=2,
	CMD_CLOSE=3,
}GimbalCmd_e;

typedef struct __gimbal_control_t
{
	uint8_t advanced1_flag;//
	uint8_t shoot_flag;		//	射击标志位
	uint8_t shoot_success_flag;//射击成功标志位
	uint8_t ramp_flag;		//斜坡标志位
	uint8_t in_flag;
	uint8_t out_flag;
	uint8_t last_mode;
	uint8_t last_cmd;
	GimbalMode_e mode;
	GimbalCmd_e  cmd;
	const MotoMeasure_t * yaw_measure;
	const MotoMeasure_t * pitch_measure;
	pid_t yaw_pid_speed;
	pid_t yaw_pid_angle;
	pid_t pitch_pid_speed;
	pid_t pitch_pid_angle;
	pid_t vision_pid_angle;
	
	float measure_d;
	float target_d;
	float alpha;
	
	int16_t yaw_current;//赋电流值
	int16_t pitch_current;//赋电流值
	
	TickType_t current_time;//当前的时间
	uint32_t shoot_time;//子弹发射时间
	uint32_t batter_time;//充电时间
	uint32_t sin_count;//产生正弦
	uint32_t ready_time;//射击准备时间
}Gimbal_t;

void control_task(void *pvParameters);
static void task_init(void);
static void set_system_target(Gimbal_t *target);
static void update_measure(Gimbal_t *update);
static void task_calculate(Gimbal_t * cal);
static void set_motor_current(int16_t moto1, int16_t moto2);
static float linearizaiton(float in , float xmin, float xmax, float ymin, float ymax);
static float quadratic_fitting(float distance);
static float three_fitting(float distance);
static float Triangle_fitting(float distance);

extern Gimbal_t g_gimbal_control;

#endif

