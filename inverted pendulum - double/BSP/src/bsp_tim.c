#include <bsp_tim.h>
#include <delay.h>


void bsp_time_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//TIM4 IC1 IC2 ±àÂëÆ÷Ä£Ê½
	TIM_TimeBaseInitStructure.TIM_ClockDivision    =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode      =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period           =   8000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler        =   1-1;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_SetCounter(TIM4,0); 
    TIM_Cmd(TIM4, ENABLE); 
}
