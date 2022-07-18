#include <bsp_tim.h>
#include <delay.h>


void bsp_time_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//频率为180M/4=45M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//频率为180M/4=45M
	
	//   TIM5_OC1/OC2 摩擦轮电机   TIM5_CH3/CH4 大摩擦轮电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=20000-1;/*  1M/20000=50HZ */
	TIM_TimeBaseInitStructure.TIM_Prescaler					=90-1;/*   90M/90=1M(1us)	*/
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 500;	
	TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	TIM_OC4Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM5 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM5 , TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5,ENABLE);
	
	
//	TIM_TimeBaseInitStructure.TIM_Period = 100-1; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM3,ENABLE); //使能定时器3
}
