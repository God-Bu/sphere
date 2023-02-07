#ifndef __UART3_H
#define __UART3_H

#include <stm32f4xx.h>

//extern float pos_x,pos_y,zangle,w_z;
//extern float O_x,O_y,O_z;
//extern u8 U2_Rceciv_error;

void USART3_Init(void);
void USART3_SendBuff(uint8_t *addr,uint8_t count_n);
void get_steering_engine_status(void);
void steering_engine_Data_Process(uint8_t *rx_buffer);
void set_steering_engine_velocity(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3);
void set_steering_engine_acceleration(uint8_t acceleration_ID1,uint8_t acceleration_ID2,uint8_t acceleration_ID3);

void set_steering_engine_velocity_four(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3,int16_t velocity_ID4);

void set_steering_engine_velocity_torque(int16_t torque_ID1,int16_t torque_ID2,int16_t torque_ID3,int16_t torque_ID4);

//void USART3_IRQHandler(void);
//u8 Action_calibtation(void);

typedef __packed struct
{
	uint16_t position;             //λ��
	int16_t velocity;              //�ٶ�
	uint16_t load;             	   //����/Ť��
	uint8_t Voltage;               //��ѹ
	uint8_t T;                     //�¶�
	uint8_t flag_Awrite;           //�첽д��ʶ
	uint8_t flag_error;            //��������ʶ �ɽ���
	uint8_t flag_move;             //����ƶ���ʶ
	uint16_t target;               //Ŀ�경���� Э���޽���
	uint16_t current;              //����
}steering_engine_status;

extern steering_engine_status Steer_engine_ID1;
extern steering_engine_status Steer_engine_ID2;
extern steering_engine_status Steer_engine_ID3;
extern steering_engine_status Steer_engine_ID4;

#endif
