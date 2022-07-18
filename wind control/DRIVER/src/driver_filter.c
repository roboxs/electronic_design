#include <driver_filter.h>

Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};


double NUM[18] = {
  0.0001539807224874,-0.001633551718237, 0.008285871521075, -0.02655137141783,
    0.05976268341326, -0.09892004512411,   0.1209510267949,  -0.1024952672175,
    0.04044670396161,  0.04044670396161,  -0.1024952672175,   0.1209510267949,
   -0.09892004512411,  0.05976268341326, -0.02655137141783, 0.008285871521075,
  -0.001633551718237,0.0001539807224874
};
double DEN[18] = {
                   1,   -12.83384996104,    77.93650732675,   -297.3335694968,
      797.9129616887,   -1597.972651706,    2472.494945198,   -3018.357074163,
      2942.681165509,   -2303.842335957,    1448.206814835,   -726.6729407251,
      287.4544421962,   -87.75235536838,    19.96212762001,      -3.188300212,
     0.3191936459711, -0.01508036852769
};

double Chebyshev50HzLPF(Filter_t *F)
{
	int i;
	for(i=17; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<18;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
	return F->filtered_value;
}

float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
		+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}
