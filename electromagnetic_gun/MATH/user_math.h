#ifndef _USER_MATH_H
#define _USER_MATH_H
#include <math.h>
// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */
#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */

#define ABS(x)   ( (x) < 0 ? -(x) : (x) )


typedef struct
{
	float period;
	float max;
	float min;
	float in;
	float out;
	float target;
}RampStr_t;//斜坡结构体

/**
 * 
 * @brief 四位16进制转化为浮点数，用联合体
 *
 */

typedef union{ 					//hex to float
	float float_data;							//浮点数
	unsigned char hex_data[4];		//四个字节十六进制数
}HexToFloat_t;

void amplitude_limit(float *object, float limit);
float add_dead_limit(float *object, float dead_lim);
short add_rc_dead_limit(short *object, float dead_lim);
void float_TO_Hex(unsigned char * pchMessage,float fdata);
void add_during_limit(float *object, float limitmin,float limitmax);
void ramp_init(RampStr_t *ramp_init, float max, float min, float period);
void ramp_calculate(RampStr_t * ramp_cal, float in);

#endif

