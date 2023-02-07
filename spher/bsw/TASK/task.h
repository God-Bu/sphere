#ifndef __TASK_H
#define __TASK_H

#include "stm32f4xx.h"
#include "sys.h"
#include "includes.h"

//#define EN_TELE_TASK 0u
//extern OS_TCB TELETaskTCB;

void StartTaskCreate(void);
void start_task(void *p_arg);
void Init_task(void *p_arg);

//void TeleCtrl_task(void *p_arg);
//extern u8 Tele_Receiv_error;


///***************LCD********************/
//#define EN_LCD_TASK 0u
//extern OS_TCB LCDTaskTCB;

//void LCD_task(void *p_arg);

///***************Cylinder***************/
//#define EN_Cylinder_TASK 0u
//extern OS_TCB CylinderTaskTCB;
//extern u8 open2_flag;
//extern int gold_num;
//void Cylinder_task(void *p_arg);

///*****************CM******************/
//#define EN_CM_TASK 0u
//#define EN_CAN1_Init 1u
//#define EN_MOTOR_INIT 1u
//extern OS_TCB CMTaskTCB;
//void CM_task(void *p_arg);

/***************SphereMove***************/
#define EN_SphereMove_TASK 1u
void SphereMove_task(void *p_arg);
float PID_Vw(float target_Position,float this_Position);




//typedef struct position
//{
//	float x;
//	float y;
//	float angle;
//}pos_t;
//extern pos_t target_pos[5];

//typedef struct line
//{
//	float k;
//	float b;
//	float angle;
//	float xst;
//}line_t;
//extern line_t Lin[4];

//extern u8 operation_step;
//u8 Get_Velt(short *v_x,short *v_y,short *v_z,u8 path);
//short PD_PosYforStop(float this_pos,float target_pos);
//short PD_PosXforStop(float this_pos,float target_pos);
//u8 P_PosYforStart(float this_pos,float target_pos,float *v_y);
//float P_PosXforStart(float this_pos,float target_pos);

#endif
