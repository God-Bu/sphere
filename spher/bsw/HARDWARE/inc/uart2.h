#ifndef __UART2_H
#define __UART2_H

#include <stm32f4xx.h>

//#define Front_Send      0xaa
//#define Front_NoSend    0xbb
//#define Front_Notask    0xf0
//#define Front_Pick      0x0f
//#define Front_ON 				0x0a
//#define Front_OFF 			0xa0
//extern u8 fmessage;
//extern u8 pick_recflag;
//extern float error_x;
void USART2_Init(void);
void UWB_Data_Process(char *rx_buffer);
//void USART2_IRQHandler(void);

typedef __packed struct
{
	float time;              //时间
	float distance_A0;              //A0距离
	float distance_A1;             	   //负载/扭力
	float distance_A2;               //电压
	float distance_A3;                     //温度
	float Acc_x;           //异步写标识
	float Acc_y;            //舵机错误标识 可解析
	float Acc_z;             //舵机移动标识
	float Gyrc_x;               //目标步数？ 协议无解释
	float Gyrc_y;              //电流
	float Gyrc_z;
	char  tag_number;
}UWB_DATA;

extern UWB_DATA UWB_data;
extern char UWB_rx_buffer[100];


#endif
