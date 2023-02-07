#include "task.h"
#include "can1.h"
#include "gpio.h"
#include "uart6.h"
#include "delay.h"
#include "includes.h"

u8 open2_flag=3;
int gold_num=5;
void Cylinder_task(void *p_arg){
	static u8 open_flag=3;
	OS_ERR err;
	p_arg = p_arg;
 
	while(1){
		if(TelCtrlData.switch_r!=open_flag)
		{
			open_flag=TelCtrlData.switch_r;
			if(open_flag==1)
				open2_flag++;
		}
		
		if(TelCtrlData.switch_l!=1)
		{
			if(TelCtrlData.switch_l==2) CB_UP();
			else CB_DOWN();
		}
		if(open2_flag==3)
		{
				CR_OPEN();
				CM_OPEN();
				CL_OPEN();
		}
		else if(open2_flag==2)
		{
				CR_CLOSE();
				CM_OPEN();
				CL_OPEN();				
		}
		else if(open2_flag==1)
		{
			if((TelCtrlData.switch_l==1)&&(operation_step==10)&&(gold_num>1))
			{
				CR_CLOSE();
				CM_OPEN();
				CL_OPEN();
			}
			else
			{
				CR_CLOSE();
				CM_OPEN();
				CL_CLOSE();
			}
		}
		else if(open2_flag==0)
		{
				CR_CLOSE();
				CM_CLOSE();
				CL_CLOSE();
		}
		else
			open2_flag=0;
		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //—” ±10ms	
	}
}
