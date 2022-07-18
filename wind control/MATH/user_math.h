#ifndef _USER_MATH_H
#define _USER_MATH_H


#define ABS(x)   ( (x) < 0 ? -(x) : (x) )

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

#endif

