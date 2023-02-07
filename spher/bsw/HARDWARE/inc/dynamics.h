#ifndef __DYNAMICS_H
#define __DYNAMICS_H

#include <stm32f4xx.h>

typedef __packed struct
{
	float torque1;
	float torque2;
	float torque3;
	float torque4;
}steering_engine_torque;

steering_engine_torque dynamic_torque(float target_u11,float target_u22,float target_u55,float target_u1,float target_u2,float target_u5);

#endif

