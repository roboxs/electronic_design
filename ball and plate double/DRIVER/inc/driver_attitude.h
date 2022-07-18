#ifndef _ATTITUDE_H
#define _ATTITUDE_H

#include <stm32f4xx.h>

typedef struct __Attitude_t
{
	float yaw;
	float pitch;
	float roll;
}Attitude_t;



static float invSqrt(float x);
static void imuComputeRotationMatrix(void);
void ahrs_updata(float ax, float ay, float az,
				 float gx, float gy, float gz,
				 float dt);
#endif

