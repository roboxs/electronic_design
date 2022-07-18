#include <user_math.h>



/****
	*@note :振幅限制（限幅）
	*@param[in] : 限幅对象
	*/
void amplitude_limit(float *object, float limit)
{
	if((*object) > limit) *object = limit;
	if((*object) < -limit) *object  = -limit;
}

/****
    *@brief 死区
    *@param[in] object      对象（误差）
    *@param[in] dead_lim	死区值
    */
float add_dead_limit(float *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
    *@brief 死区
    *@param[in] object      对象（误差）
    *@param[in] dead_lim	死区值
    */
short add_rc_dead_limit(short *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
	*@note :振幅限制（限幅）
	*@param[in] : 限幅对象
	*/
void add_during_limit(float *object, float limitmin,float limitmax)
{
	if((*object) > limitmax) *object = limitmax;
	else if((*object) < limitmin) *object  = limitmin;
}

//斜坡初始化
void ramp_init(RampStr_t *ramp_init, float max, float min, float period)
{
	ramp_init->max = max;
	ramp_init->min = min;
	ramp_init->period = period;
	ramp_init->in = 0;
	ramp_init->out =min;
}



//斜坡计算
void ramp_calculate(RampStr_t * ramp_cal, float in)
{
	ramp_cal->in = in;
	ramp_cal->out += ramp_cal->in * ramp_cal->period;
	
	if(ramp_cal->out >= ramp_cal->in) 
		ramp_cal->out =ramp_cal->in;
	
	if(ramp_cal->out > ramp_cal->max)
	{
		ramp_cal->out =ramp_cal->max;
	}
	else if(ramp_cal->out < ramp_cal->min)
	{
		ramp_cal->out = ramp_cal->min;
	}
}

