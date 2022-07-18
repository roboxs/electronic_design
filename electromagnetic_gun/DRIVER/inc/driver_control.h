#ifndef _DRIVER_CONTROL_H
#define _DRIVER_CONTROL_H

#include <stm32f4xx.h>
#include <driver_filter.h>


/****************************PID�ṹ��*************************************/

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,//λ��ʽ
    DELTA_PID,	 //����ʽ
	ANALOG_PID,  //ģ��
};

enum
{
	ANGLE_LOOP = 0,
	SPEED_LOOP = 1,
	DOUBLE_LOOP = 2,
};

typedef struct 
{
	float err_max;//pid��������
	float err_min;//pid���������
	float umax;//ִ�����������ֵ
	float umin;//ִ��������С��ֵ
	float param; //���ִ���������ò���
	
}pid_integral_t;//pid���ִ���ṹ��


typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
	float target;				//Ŀ��ֵ
	float measure;				//����ֵ
    float err[3];				//���
	
    
    float pout;							
    float iout;							
    float dout[3];		
	float dout_lpf;				//��ͨ�˲����΢����
	Butter_BufferData bufferdata;//��ͨ�˲������ݱ���
	
	float alpha;				//����ȫ΢��ϵ��
	
	float integ;				//����
	float deriv;				//΢��
	float dt;					//��t
	
	float analog_out;			//ģ�����
    
    float pos_out;				//����λ��ʽ���

    float delta_u;				//��������ֵ
    float delta_out;			//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;
    
	pid_integral_t integral;
	float deadband;				//����
    uint32_t pid_mode;
	uint32_t feedback_loop;			//������
    uint32_t MaxOutput;				//����޷�
    uint32_t IntegralLimit;			//�����޷�
    
    void (*f_param_init)(struct __pid_t *pid,  //PID������ʼ��
                    uint32_t pid_mode,
					uint32_t feedback_loop,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid���������޸�

}pid_t;

/******************************PID������ʼ��**************************************/

void pid_init_all(void);

float pid_calculate(pid_t * pidInner,pid_t * pidOuter);
float single_pid_calculate(pid_t * pid_cal);
		
void pid_struct_init(
    pid_t* pid,
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);
	
void pid_reset(pid_t	*pid, float kp, float ki, float kd);


/*******************************PID��������***************************************/

	
#endif

