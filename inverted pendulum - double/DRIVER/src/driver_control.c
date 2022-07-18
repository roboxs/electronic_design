#include <driver_control.h>
#include <user_math.h>
#include <driver_filter.h>





void pid_init_all(void)
{				
//	pid_struct_init(&push_motor_speed, POSITION_PID, SPEED_LOOP, PUSH_MOTOR_SPEED_MAXOUT, PUSH_MOTOR_SPEED_INTEGRATION_LIMIT,
//		PUSH_MOTOR_P, PUSH_MOTOR_I, PUSH_MOTOR_D);
	;
}




/****
    *@brief �޷�
    *@param[in] object   ��Ҫ�޷�����
    *@param[in] abs_max	 �޷�ֵ
    */
void abs_limit(float *object, float abs_max)
{
    if(*object > abs_max)  *object =  abs_max;
    if(*object < -abs_max) *object = -abs_max;
}


/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
void dead_limit(float *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
}
/****
    *@brief ���ַ���
    *@param[in] ��ǰ����ֵ  ���ֵ ���ֽṹ��
    *@param[in] 
    */

float integral_separation(float *integral, float *err, pid_integral_t* integral_sep)
{
	if(ABS(*err) > integral_sep->err_min)
	{
		integral_sep->param = 0;
	}
	else 
	{
		integral_sep->param = 1;
		*integral += (*err);
	}
	return integral_sep->param;
}

/****
    *@brief �����
    *@param[in] ��ǰ����ֵ  ���ֵ ���ֽṹ��
    *@param[in] 
    */ 
float integral_alter(float *integral, float *err, pid_integral_t* integral_sep)
{
	float index=0.0;
	if( (ABS(*err) ) > integral_sep->err_max)  index =0.0;
	else if( (ABS(*err)) < integral_sep->err_min) 
	{
		index =1.0;
		*integral += *err;
	}
	else 
	{
		index = (integral_sep->err_max-ABS(*err))/(integral_sep->err_max - integral_sep->err_min);
		*integral += *err;
	}
	integral_sep->param = index;
	return index;
}

/****
	*@brief �����ֱ���
	*@param[in] ��ǰ����ֵ ��� ���ֽṹ��
	*@param[out] ���ֽṹ��Ĳ���
	*/

float anti_windup(float *measure, float *integral, float *err, pid_integral_t * integral_anti)
{
	float index;
	if(*measure > integral_anti->umax)//����ִ�����������ֵ
	{
		if( ABS(*err) > integral_anti->err_max) //���ַ���
		{
			index =0;
		}
		else {
			index =1;
			if((*err) < 0) *integral+=*err;//ֻ���۷���ƫ��
		}
	}
	else if(*measure < integral_anti->umin)//����ִ��������С��ֵ
	{
		if(ABS(*err) > integral_anti->err_max) //���ַ���
		{
			index = 0;
		}
		else{
			index =1;
			if((*err) > 0) *integral +=*err;//ֻ��������ƫ��
		}
	}
	else{
		if(ABS(*err) > integral_anti->err_max)
		{
			index =0;
		}else{
			index =1; 
			*integral+=*err;
		}
	}
	integral_anti->param = index;
	return index;
}


/****
	*@brief pid������ʼ������ �ڲ�����
	*@param[in] pid��ֵ
	*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
	pid->feedback_loop = loop;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}

/****
	*@brief pid�����޸ĺ��� �ڲ�����
	*@param[in] KP KI KD
	*/
void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/****
	*@brief pid�ṹ���ʼ��
	*@param[in] KP KI KD
	*/

void pid_struct_init(
    pid_t* pid,
    uint32_t mode,
	uint32_t loop,
    uint32_t maxout,
    uint32_t intergral_limit,

    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*��ʼ������ָ��*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset  = pid_reset;;
		
    pid->f_param_init(pid, mode,loop, maxout, intergral_limit, kp, ki, kd);//��ʼ��PID�Ļ�������
	
}

/****
	*@brief ����pid���㺯��
	*@param[in] pid�ṹ��
	*/
float single_pid_calculate(pid_t * pid_cal)
{
	if(pid_cal->pid_mode == POSITION_PID)
	{
		pid_cal->err[NOW] = pid_cal->target - pid_cal->measure;//��ǰ���
		
		pid_cal->pout = pid_cal->p * pid_cal->err[NOW];
		/*������Ľ�*/
		pid_cal->iout += pid_cal->i * pid_cal->err[NOW];
		if( ABS (pid_cal->err[NOW]) != 0)//���л��ַ���
		{
			if( ABS(pid_cal->err[NOW]) > pid_cal->integral.err_min)//�ڸ�������ʹ��PD����
			{
				pid_cal->integral.param = 0.0f;
			}
			else//Ҳ�����޸�ʹ�÷ֶ�ʽ���ֿ���
			{
				pid_cal->integral.param = 1.0f;
			}
		}
		abs_limit(&(pid_cal->iout),pid_cal->IntegralLimit);//�����޷�
		/*΢����Ľ�*/
		pid_cal->dout[NOW] = pid_cal->d * (1 - pid_cal->alpha)*(pid_cal->err[NOW] - pid_cal->err[LAST]) + pid_cal->alpha * pid_cal->dout[LAST];//΢������е�ͨ�˲�
		pid_cal->pos_out = pid_cal->pout + pid_cal->integral.param * pid_cal->iout + pid_cal->dout[NOW];
		abs_limit(&(pid_cal->pos_out), pid_cal->MaxOutput);//����޷�
		pid_cal->err[LAST] = pid_cal->err[NOW];
		pid_cal->dout[LAST] = pid_cal->dout[NOW];
	}
	return pid_cal->pos_out;
}


/****
	*@brief pid���㺯��
	*@param[in] �ڻ�pid  �⻷pid
	*/
float pid_calculate(pid_t * pidInner,pid_t * pidOuter)
{
    if(pidInner->pid_mode == POSITION_PID) //λ��ʽPID
    {
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //��ǰ�Ƕ����
//		dead_limit(&(pidOuter->err[NOW]),DEADBAND);								 //��������
		pidOuter->pout  = pidOuter->p *  pidOuter->err[NOW];
		/*�������*/
		if(pidOuter->integral.err_min == 0)	
			pidOuter->iout +=  pidOuter->i * pidOuter->err[NOW];
		else 
		{
			if(pidOuter->err[NOW] < pidOuter->integral.err_min)
				pidOuter->iout += pidOuter->i * pidOuter->err[NOW];
			else
				pidOuter->iout =0;
		}
		//΢�����
		pidOuter->dout[NOW]  = pidOuter->d *(pidOuter->err[NOW]-pidOuter->err[LAST]);//ԭʼ΢����
		//�����޷�
		abs_limit(&(pidOuter->iout),pidOuter->IntegralLimit);
		pidOuter->pos_out = (pidOuter->pout + pidOuter->iout +  pidOuter->dout[NOW]);
		//������޷�
		abs_limit(&(pidOuter->pos_out),pidOuter->MaxOutput);
		pidOuter->err[LAST]=pidOuter->err[NOW];

		
		
		/*****************************  �ٶ��ڻ�  *****************************/
		if(pidInner->feedback_loop == DOUBLE_LOOP || pidInner->feedback_loop == ANGLE_LOOP)
		{
			pidInner->err[NOW] = pidOuter->pos_out - pidInner->measure;	 //��ǰ˫���ٶ����ֵ
		}
		else if(pidInner->feedback_loop == SPEED_LOOP)
		{
			pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ�ٶ����ֵ
		}
		//��������
		if(pidInner->deadband != RESET)		dead_limit(&(pidInner->err[NOW]),pidInner->deadband);
		pidInner->pout  = pidInner->p *  pidInner->err[NOW];			 
		pidInner->iout += pidInner->i *  pidInner->err[NOW];		 	
		//΢������д���
		pidInner->dout[NOW]  =pidInner->d *(pidInner->err[NOW]-pidInner->err[LAST]);
//		pidInner->dout_lpf = Control_Device_LPF(pidInner->dout[NOW],&pidInner->bufferdata,&Control_Device_Div_LPF_Parameter);//20hz������˹��ͨ�˲�
		//�����޷�
		abs_limit(&(pidInner->iout),pidInner->IntegralLimit);		
		pidInner->pos_out = (pidInner->pout + pidInner->iout +  pidInner->dout[NOW]);	//λ��ʽPID���
		//λ��ʽPID����޷�
		abs_limit(&(pidInner->pos_out),pidInner->MaxOutput);		
			
		pidInner->err[LAST]=pidInner->err[NOW];
			
    }

		
		
    else if(pidInner->pid_mode == DELTA_PID)//����ʽPID
    {
		
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;	 //��ǰ�Ƕ����ֵ
			
		pidOuter->pout = pidOuter->p * (pidOuter->err[NOW] - pidOuter->err[LAST]);
		pidOuter->iout = pidOuter->i *  pidOuter->err[NOW];
		pidOuter->dout[NOW] = pidOuter->d * (pidOuter->err[NOW] - 2*pidOuter->err[LAST] + pidOuter->err[LLAST]);
        
		abs_limit(&(pidOuter->iout), pidOuter->IntegralLimit);
		pidOuter->delta_u = pidOuter->pout + pidOuter->iout + pidOuter->dout[NOW];
		pidOuter->delta_out = pidOuter->last_delta_out + pidOuter->delta_u;
		abs_limit(&(pidOuter->delta_out), pidOuter->MaxOutput);
		pidOuter->last_delta_out = pidOuter->delta_out;	//�����ϴ�����ʽ��
			
		pidOuter->err[LLAST]=pidOuter->err[LAST];
		pidOuter->err[LAST]=pidOuter->err[NOW];
			
			/*****************************  �ٶ��ڻ�  *****************************/
		pidInner->err[NOW] = pidOuter->delta_out - pidInner->measure;	 //��ǰ�ٶ����ֵ
			//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ�ٶ����ֵ
			
		pidInner->pout = pidInner->p * (pidInner->err[NOW] - pidInner->err[LAST]);
		pidInner->iout = pidInner->i *  pidInner->err[NOW];
		pidInner->dout[NOW] = pidInner->d * (pidInner->err[NOW] - 2*pidInner->err[LAST] + pidInner->err[LLAST]);
        
		abs_limit(&(pidInner->iout), pidInner->IntegralLimit);
		pidInner->delta_u   = pidInner->pout           + pidInner->iout   + pidInner->dout[NOW];
		pidInner->delta_out = pidInner->last_delta_out + pidInner->delta_u;
		abs_limit(&(pidInner->delta_out), pidInner->MaxOutput);
		pidInner->last_delta_out = pidInner->delta_out;	//�����ϴ�����ʽ��ֵ
			
		pidInner->err[LLAST] = pidInner->err[LAST];
		pidInner->err[LAST]  = pidInner->err[NOW];
			
    }
		
		
		
	else if(pidInner->pid_mode == ANALOG_PID)//ģ����PID
	{			
		/*****************************  �Ƕ��⻷  *****************************/
		pidOuter->err[NOW] = pidOuter->target - pidOuter->measure;//�Ƕ����
			
		pidOuter->integ += pidOuter->err[NOW]*pidOuter->dt;		 //ģ�����
		abs_limit(&(pidOuter->integ),pidOuter->IntegralLimit); //�����޷�
		pidOuter->deriv  = (pidOuter->err[NOW] - pidOuter->err[LAST])/pidOuter->dt;//ģ��΢��
			
		pidOuter->pout = pidOuter->p * pidOuter->err[NOW];
		pidOuter->iout = pidOuter->i * pidOuter->integ;
		pidOuter->dout[NOW] = pidOuter->d * pidOuter->deriv;
			
		pidOuter->analog_out = pidOuter->pout+pidOuter->iout+pidOuter->dout[NOW];//ģ�����
		abs_limit(&(pidOuter->analog_out),pidOuter->MaxOutput);
			
		pidOuter->err[LAST]  = pidOuter->err[NOW];
			
			
		/*****************************  �ٶ��ڻ�  *****************************/
		pidInner->err[NOW] = pidOuter->analog_out - pidInner->measure;//�ٶ����
		//pidInner->err[NOW] = pidInner->target - pidInner->measure;	 //��ǰ���ֵ
			
		pidInner->integ += pidInner->err[NOW]*pidInner->dt;		 //ģ�����
		abs_limit(&(pidInner->integ),pidInner->IntegralLimit); //�����޷�
		pidInner->deriv  = (pidInner->err[NOW] - pidInner->err[LAST])/pidInner->dt;//ģ��΢��
			
		pidInner->pout = pidInner->p * pidInner->err[NOW];
		pidInner->iout = pidInner->i * pidInner->integ;
		pidInner->dout[NOW] = pidInner->d * pidInner->deriv;
			
		pidInner->analog_out = pidInner->pout+pidInner->iout+pidInner->dout[NOW];//ģ�����
		abs_limit(&(pidInner->analog_out),pidInner->MaxOutput);
			
		pidInner->err[LAST]  = pidInner->err[NOW];
	}
		
		
	if      (pidInner->pid_mode == POSITION_PID) 
	{
		if(pidInner->feedback_loop == SPEED_LOOP || pidInner->feedback_loop == DOUBLE_LOOP) 
			return pidInner->pos_out;
		else if(pidInner->feedback_loop == ANGLE_LOOP) 
			return pidOuter->pos_out;
	}
	else if (pidInner->pid_mode == DELTA_PID)    return pidInner->delta_out;
	else if	(pidInner->pid_mode == ANALOG_PID)   return pidInner->analog_out;
	return 0;
}



