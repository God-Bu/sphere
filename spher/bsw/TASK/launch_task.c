#include "task.h"
#include "delay.h"
#include "can1.h"
#include "includes.h"

void Launch_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0x64,5000, 0);
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0x74,5000, 0);
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0x84,5000, 0);
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0xa4,5000, 0);
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0xb4,5000, 0);
		CAN_RoboModule_DRV_PWM_Velocity_Mode(0xc4,5000, 0);
		OSTimeDly(20,OS_OPT_TIME_PERIODIC,&err); //—” ±20ms	
	}
}
