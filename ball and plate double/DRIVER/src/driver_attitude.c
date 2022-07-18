#include <driver_attitude.h>
#include <user_math.h>
#include <math.h>

//定义姿态变量
Attitude_t g_attitude_angle;
//开方分之一
static float invSqrt(float x)
{
	return 1.0f/sqrtf(x);
}

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//四元数
static float rMat[3][3];//四元数的旋转矩阵	

//姿态矩阵更新,b系转换至n系
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
	float acc_length_recip,q_length_recip;//长度倒数
	float ex,ey,ez;
	static float ex_integral = 0,ey_integral = 0,ez_integral = 0;
	float q0_last, q1_last, q2_last;
	
	//1.加速度归一化
	acc_length_recip = invSqrt(ax*ax + ay*ay + az*az);
	ax *= acc_length_recip;
	ay *= acc_length_recip;
	az *= acc_length_recip;
	//2.加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
	ex = (ay * rMat[2][2] - az * rMat[2][1]);
	ey = (az * rMat[2][0] - ax * rMat[2][2]);
	ez = (ax * rMat[2][1] - ay * rMat[2][0]);
	//3.利用PI消除当前的误差值
	ex_integral += ex * KI_ACC;
	ey_integral += ey * KI_ACC;
	ez_integral += ez * KI_ACC;
	
	gx += ex * KP_ACC + ex_integral;//互补滤波
	gy += ex * KP_ACC + ey_integral;
	gz += ex * KP_ACC + ez_integral;
	//4.一阶龙格库塔法更新四元数
	q0_last = q0;
	q1_last = q1;
	q2_last = q2;
	
	q0 += 0.5f * (-q1_last * gx - q2_last * gy - q3 * gz) * dt;
	q1 += 0.5f * ( q0_last * gx + q2_last * gz - q3 * gy) * dt;
	q2 += 0.5f * ( q0_last * gy - q1_last * gz + q3 * gx) * dt;
	q3 += 0.5f * ( q0_last * gz + q1_last * gy - q2 * gx) * dt;
	//5.单位化四元数
	q_length_recip = invSqrt(q0*q0 + q1*q1 + q2*q2 +q3*q3);
	q0 *= q_length_recip;
	q1 *= q_length_recip;
	q2 *= q_length_recip;
	q3 *= q_length_recip;
	//6.更新姿态矩阵
	rotation_matrix_updata();
	//7.更新欧拉角,方向余弦的矩阵的旋转顺序不一样，该公式也就不一样
	g_attitude_angle.roll = atan2_approx(rMat[2][1], rMat[2][2]) * RAD2DEG;
	g_attitude_angle.pitch = (0.5f * M_PIf) - acos_approx(-rMat[2][0]) * RAD2DEG;
	g_attitude_angle.yaw = atan2_approx(rMat[1][0], rMat[0][0]) * RAD2DEG;
}

