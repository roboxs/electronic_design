#ifndef _USER_MATH_H
#define _USER_MATH_H

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */
#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */

#define MIN(a, b) 	(((a) < (b)) ? (a) : (b))
#define MAX(a, b) 	(((a) > (b)) ? (a) : (b))
#define ABS(x)  ( (x) < 0 ? -(x) : (x) )

/**
 * 
 * @brief 四位16进制转化为浮点数，用联合体
 *
 */

typedef union{ 					//hex to float
	float float_data;							//浮点数
	unsigned char hex_data[4];		//四个字节十六进制数
}HexToFloat_t;

float sqf(float x);
void amplitude_limit(float *object, float limit);
float add_dead_limit(float *object, float dead_lim);
short add_rc_dead_limit(short *object, float dead_lim);
void float_TO_Hex(unsigned char * pchMessage,float fdata);
float atan2_approx(float y, float x);
float acos_approx(float x);

#endif

