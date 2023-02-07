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
	float time;              //ʱ��
	float distance_A0;              //A0����
	float distance_A1;             	   //����/Ť��
	float distance_A2;               //��ѹ
	float distance_A3;                     //�¶�
	float Acc_x;           //�첽д��ʶ
	float Acc_y;            //��������ʶ �ɽ���
	float Acc_z;             //����ƶ���ʶ
	float Gyrc_x;               //Ŀ�경���� Э���޽���
	float Gyrc_y;              //����
	float Gyrc_z;
	char  tag_number;
}UWB_DATA;

extern UWB_DATA UWB_data;
extern char UWB_rx_buffer[100];


#endif
