#ifndef __UART6_H
#define __UART6_H

#include <stm32f4xx.h>

//typedef __packed struct
//{
//	int8_t switch_l;
//	int8_t switch_r;
//	uint8_t mouse_l;
//	uint8_t mouse_r;
//	int16_t right_x;
//	int16_t right_y;
//	int16_t left_x;
//	int16_t left_y;
//	int16_t mouse_x;
//	int16_t mouse_y;
//	int16_t mouse_z;
//	uint16_t  key;
//}Tel_Ctrl_t;

//extern Tel_Ctrl_t TelCtrlData;
//extern u8 Tele_Receiv_error;
extern uint8_t sbus_rx_buffer[35];
void USART6_Config(void);
void Data_Process(uint8_t *sbus_rx_buffer);
float Angle_Y_Process(float Gyrc, float Angle);


//void USART6_IRQHandler(void);

typedef __packed struct
{
	float Acc_x;              //X加速度
	float Acc_y;
	float Acc_z;
	float T;
}Acc;

typedef __packed struct
{
	float Gyrc_x;              //X角速度
	float Gyrc_y;
	float Gyrc_z;
	float T;
}Gyrc;

typedef __packed struct
{
	float Angle_x;              //X角速度
	float Angle_y;
	float Angle_z;
	float T;
}Angle;

typedef __packed struct
{
	float Acc_angle_x;              //X角加速度
	float Acc_angle_y;
	float Acc_angle_z;
}Acc_angle;

extern Acc Acc_Data;
extern Gyrc Gyrc_Data;
extern Angle Angle_Data;
extern Acc_angle Acc_angle_Data;

extern double angle_yaw;  

float Acc_angle_produce_x(float Gyrc_x,float time_count);
float Acc_angle_produce_y(float Gyrc_y,float time_count);
float Acc_angle_produce_z(float Gyrc_z,float time_count);

#endif
