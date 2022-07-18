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

