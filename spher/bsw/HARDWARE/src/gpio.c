#include "gpio.h"

void GPIO_Config(void)
{
    GPIO_InitTypeDef  gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
		//继电器&LCD_ROM引脚
		gpio.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6; 
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOE,&gpio);

		gpio.GPIO_Pin = GPIO_Pin_5; 
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOE,&gpio);
	
		//LED引脚
		gpio.GPIO_Pin = GPIO_Pin_9; 
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOB,&gpio); 
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	
		//LCD引脚
		gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_13;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		GPIO_Init(GPIOC,&gpio); 
		lcd_led()=1;//开背光灯
		
		//按键引脚
		gpio.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_2; 
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_PuPd = GPIO_PuPd_DOWN; 
		GPIO_Init(GPIOA,&gpio);

}
