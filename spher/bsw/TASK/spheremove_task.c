#include "task.h"
#include "includes.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "pstwo.h"
#include "delay.h"
#include "sys.h"
#include "kinematic.h"
#include "uart6.h"
#include "dynamics.h"

int16_t target_velocity_ID1;
int16_t target_velocity_ID2;
int16_t target_velocity_ID3;
int16_t target_velocity_ID4;

float target_angle_yaw=-45.0f;

float VX=0;
float VY=0;
float WZ=0;

steering_engine_torque torque;

void SphereMove_task(void *p_arg){	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		PS2_Receive();
		get_steering_engine_status();            
		torque=dynamic_torque(0,0,0,-1,-1,0.3);
		
		set_steering_engine_velocity_torque(torque.torque1,torque.torque2,torque.torque3,torque.torque4);                            //开启定时器读取舵机状态  两次控制信息之间必须有间隔
		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //10ms控制周期	
	}
}


