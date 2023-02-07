#ifndef __GPIO_H__
#define __GPIO_H__

#include <stm32f4xx.h>
#include "sys.h"
#define LED1 PBout(9)

#define lcd_cs()    PCout(3)
#define lcd_reset() PCout(2)
#define lcd_rs()    PCout(0)
#define lcd_clk()   PCout(15)
#define lcd_data()  PCout(1)
#define lcd_led()   PCout(14)    
#define lcd_so()    PEin(5)
#define lcd_si()    PEout(4)
#define lcd_sclk()  PEout(6)
#define lcd_scs()   PCout(13)

#define Button_Left() PAin(2)
#define Button_Right() PAin(0)

void GPIO_Config(void);
#define CR_OPEN() 		GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define CR_CLOSE() 		GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define CL_OPEN() 		GPIO_SetBits(GPIOE,GPIO_Pin_0)
#define CL_CLOSE() 		GPIO_ResetBits(GPIOE,GPIO_Pin_0)
#define CM_OPEN()     GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define CM_CLOSE()    GPIO_ResetBits(GPIOE,GPIO_Pin_3)
#define CB_DOWN()     GPIO_ResetBits(GPIOE,GPIO_Pin_1)
#define CB_UP()       GPIO_SetBits(GPIOE,GPIO_Pin_1)

#endif
