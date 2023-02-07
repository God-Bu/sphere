#include "uart1.h"
#include "sys.h"
//#include "gpio.h"
//////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

void USART1_Init(void){	//与uwb的通讯
	
	USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 

	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	
	gpio.GPIO_OType = GPIO_OType_PP; 
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio); 

	nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    USART_DeInit(USART1);
    usart.USART_BaudRate = 115200;                //USART1 38400bps
    usart.USART_WordLength = USART_WordLength_8b;//字长8B
    usart.USART_StopBits = USART_StopBits_1;//一个停止位
    usart.USART_Parity = USART_Parity_No;//奇偶校验位
    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//串口模式  接受模式
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart);
	USART_Cmd(USART1,ENABLE);
		
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);	
	
}

//u8 U1_Rceciv_error=1;                   //接收失败的次数
//u8 CCD_Len=0,Crossed_flag=0;
//u8 CCD_Pos=0;
//void USART1_IRQHandler(void) {  	
//		 static u8 count=0;
//		 static u8 ch,temp[3];    
//	   OSIntEnter();
//		 if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET){
//         USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
//				 ch=USART_ReceiveData(USART1);
//				 switch(count){
//              case 0:
//                  if(ch==0x0a)
//                      count++;
//                  else
//                      count=0;
//                  break;
//							case 1:
//                  temp[0]=ch;
//									count++;
//                  break;
//							case 2:
//                  temp[1]=ch;
//									count++;
//                  break;
//							case 3:
//									temp[2]=ch;
//									count++;
//                  break;
//						  case 4:
//					  			if(ch==0x0d)
//									{
//										CCD_Pos=temp[0];
//										CCD_Len=temp[1];
//										Crossed_flag=temp[2];
//										U1_Rceciv_error=0;
//									}
//									count=0;
//                  break;   
//							default:
//									count=0;
//							break;
//					}
//		 }									
//		 OSIntExit(); 
//}


void USART1_IRQHandler(void) {
	static u8 ch;    
	OSIntEnter();
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		ch=USART_ReceiveData(USART1);
	}								
	OSIntExit(); 
}
