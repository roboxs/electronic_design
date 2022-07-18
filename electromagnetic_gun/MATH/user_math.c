#include <user_math.h>



/****
	*@note :������ƣ��޷���
	*@param[in] : �޷�����
	*/
void amplitude_limit(float *object, float limit)
{
	if((*object) > limit) *object = limit;
	if((*object) < -limit) *object  = -limit;
}

/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
float add_dead_limit(float *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
short add_rc_dead_limit(short *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
	*@note :������ƣ��޷���
	*@param[in] : �޷�����
	*/
void add_during_limit(float *object, float limitmin,float limitmax)
{
	if((*object) > limitmax) *object = limitmax;
	else if((*object) < limitmin) *object  = limitmin;
}

//б�³�ʼ��
void ramp_init(RampStr_t *ramp_init, float max, float min, float period)
{
	ramp_init->max = max;
	ramp_init->min = min;
	ramp_init->period = period;
	ramp_init->in = 0;
	ramp_init->out =min;
}



//б�¼���
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

