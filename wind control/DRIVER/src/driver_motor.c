#include <driver_motor.h>

void driver_wind_motor(float xspeed,float yspeed)
{
	float moto1,moto2,moto3,moto4;
	if(xspeed > 0) //xÖá
	{
		moto2 = xspeed;
		moto3 = 0;
	}
	else if(xspeed < 0) 
	{
		moto2 = 0;
		moto3 = -xspeed;
	}
	else
	{
		moto2 = 0;
		moto3 = 0;
	}
	
	if(yspeed > 0) //yÖá
	{
		moto1 = yspeed;
		moto4 = 0;
	}
	else if(yspeed < 0) 
	{
		moto1 = 0;
		moto4 = -yspeed;
	}
	else
	{
		moto1 = 0;
		moto4 = 0;
	}
	TIM_SetCompare1(TIM5,moto1);
	TIM_SetCompare2(TIM5,moto2);
	TIM_SetCompare3(TIM5,moto3);
	TIM_SetCompare4(TIM5,moto4);
}
