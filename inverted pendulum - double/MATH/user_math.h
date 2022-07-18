#ifndef _USER_MATH_H
#define _USER_MATH_H


#define ABS(x)   ( (x) < 0 ? -(x) : (x) )

/**
 * 
 * @brief ��λ16����ת��Ϊ����������������
 *
 */

typedef union{ 					//hex to float
	float float_data;							//������
	unsigned char hex_data[4];		//�ĸ��ֽ�ʮ��������
}HexToFloat_t;

void amplitude_limit(float *object, float limit);
float add_dead_limit(float *object, float dead_lim);
short add_rc_dead_limit(short *object, float dead_lim);
void float_TO_Hex(unsigned char * pchMessage,float fdata);

#endif

