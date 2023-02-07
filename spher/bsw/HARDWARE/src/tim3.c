#include "tim3.h"
#include "sys.h"
#include "uart3.h"
//////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif



void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef  nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	tim.TIM_Prescaler = 8400-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 5-1;
    TIM_TimeBaseInit(TIM3,&tim);
	
	nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );
	//TIM_Cmd(TIM3, ENABLE);
}


void TIM3_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)               //��ʱ����ʱ�жϷ���
	{
		TIM_Cmd(TIM3, DISABLE);                                     //�رյȴ���һ�ο���
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		//get_steering_engine_status();
		
		
	}
	OSIntExit(); 
}
