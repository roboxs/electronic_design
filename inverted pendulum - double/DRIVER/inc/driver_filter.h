#ifndef _DRIVER_FILTER_H
#define _DRIVER_FILTER_H

typedef struct{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
}Filter_t;


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;

double Chebyshev50HzLPF(Filter_t *F);
float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


//��������
extern Butter_Parameter Control_Device_Div_LPF_Parameter;

#endif

