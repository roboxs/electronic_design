#include <driver_attitude.h>
#include <user_math.h>
#include <math.h>

//������̬����
Attitude_t g_attitude_angle;
//������֮һ
static float invSqrt(float x)
{
	return 1.0f/sqrtf(x);
}

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//��Ԫ��
static float rMat[3][3];//��Ԫ������ת����	

//��̬�������,bϵת����nϵ
static void rotation_matrix_updata(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

#define KP_ACC 0.995f
#define KI_ACC 0.005f

void ahrs_updata(float ax, float ay, float az,
				 float gx, float gy, float gz,
				 float dt)
{
	float acc_length_recip,q_length_recip;//���ȵ���
	float ex,ey,ez;
	static float ex_integral = 0,ey_integral = 0,ez_integral = 0;
	float q0_last, q1_last, q2_last;
	
	//1.���ٶȹ�һ��
	acc_length_recip = invSqrt(ax*ax + ay*ay + az*az);
	ax *= acc_length_recip;
	ay *= acc_length_recip;
	az *= acc_length_recip;
	//2.���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���
	ex = (ay * rMat[2][2] - az * rMat[2][1]);
	ey = (az * rMat[2][0] - ax * rMat[2][2]);
	ez = (ax * rMat[2][1] - ay * rMat[2][0]);
	//3.����PI������ǰ�����ֵ
	ex_integral += ex * KI_ACC;
	ey_integral += ey * KI_ACC;
	ez_integral += ez * KI_ACC;
	
	gx += ex * KP_ACC + ex_integral;//�����˲�
	gy += ex * KP_ACC + ey_integral;
	gz += ex * KP_ACC + ez_integral;
	//4.һ�����������������Ԫ��
	q0_last = q0;
	q1_last = q1;
	q2_last = q2;
	
	q0 += 0.5f * (-q1_last * gx - q2_last * gy - q3 * gz) * dt;
	q1 += 0.5f * ( q0_last * gx + q2_last * gz - q3 * gy) * dt;
	q2 += 0.5f * ( q0_last * gy - q1_last * gz + q3 * gx) * dt;
	q3 += 0.5f * ( q0_last * gz + q1_last * gy - q2 * gx) * dt;
	//5.��λ����Ԫ��
	q_length_recip = invSqrt(q0*q0 + q1*q1 + q2*q2 +q3*q3);
	q0 *= q_length_recip;
	q1 *= q_length_recip;
	q2 *= q_length_recip;
	q3 *= q_length_recip;
	//6.������̬����
	rotation_matrix_updata();
	//7.����ŷ����,�������ҵľ������ת˳��һ�����ù�ʽҲ�Ͳ�һ��
	g_attitude_angle.roll = atan2_approx(rMat[2][1], rMat[2][2]) * RAD2DEG;
	g_attitude_angle.pitch = (0.5f * M_PIf) - acos_approx(-rMat[2][0]) * RAD2DEG;
	g_attitude_angle.yaw = atan2_approx(rMat[1][0], rMat[0][0]) * RAD2DEG;
}

