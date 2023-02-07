#include "tim2.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

uint16_t time_count=0;



void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef  nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	tim.TIM_Prescaler = 840-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 10000-1;
    TIM_TimeBaseInit(TIM2,&tim);
	
	nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
	TIM_Cmd(TIM2, ENABLE);
}


void TIM2_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)               //计时器定时中断发生
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//USART_SendData(USART3,0xaa);
		time_count++;
	}
	OSIntExit(); 
}

