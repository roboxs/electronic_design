#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H
#include <stm32f4xx.h>
#include <driver_control.h>

#define WIND_XSPEED_KP 150
#define WIND_XSPEED_KI 0.1
#define WIND_XSPEED_KD 0
#define WIND_XSPEED_MAXOUT 20000
#define WIND_XSPEED_INTEGRATION_LIMIT 3000
#define WIND_XANGLE_KP -10
#define WIND_XANGLE_KI 0
#define WIND_XANGLE_KD 0
#define WIND_XANGLE_MAXOUT 2000
#define WIND_XANGLE_INTEGRATION_LIMIT 0

#define WIND_YSPEED_KP 150
#define WIND_YSPEED_KI 0.1
#define WIND_YSPEED_KD 0
#define WIND_YSPEED_MAXOUT 20000
#define WIND_YSPEED_INTEGRATION_LIMIT 3000
#define WIND_YANGLE_KP -10
#define WIND_YANGLE_KI 0
#define WIND_YANGLE_KD 0
#define WIND_YANGLE_MAXOUT 1000
#define WIND_YANGLE_INTEGRATION_LIMIT 1000

#define WIND_ANGLE_LIMIT 30

#define TASK_CONTROL_PERIOD 10		//������������
#define SIN_PERIOD			180.0f//sin�������ڣ��Ϳ��������������

typedef enum _wind_mode_e
{
	WIND_STOP = 0,//��ֹģʽ
	WIND_LINE = 1,//����ģʽ
	WIND_CYCLE =2,//��Բģʽ
	WIND_CLOSE =3,//�ػ�ģʽ
}WindMode_e;


typedef struct __wind_control_t
{
	WindMode_e mode;
	
	float line_length;//ֱ�߳���
	float line_angle;//ֱ�߽Ƕ�����
	float cycle_r;//Բ�뾶����
	
	float sin_calculate_x;
	float sin_calculate_y;
	pid_t wind_pid_xspeed;//x��˫��
	pid_t wind_pid_xangle;
	
	pid_t wind_pid_yspeed;//y��˫��
	pid_t wind_pid_yangle;
	
	float offset_xangle;
	float offset_yangle;
	uint32_t sin_count;
}WindControl_t;	



static void control_task_init(void);
void control_task(void *pvParameters);
static void wind_set_mode(WindControl_t *mode);
static void set_wind_target(WindControl_t *target);
static void task_control_update(WindControl_t *update);
static void control_pid_calculate(WindControl_t *calculate);
static float linearizaiton(float in , float xmin, float xmax, float ymin, float ymax);

#endif

