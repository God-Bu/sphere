#include "task.h"
#include "delay.h"
#include "can1.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "uart6.h"
#include "gpio.h"
#include "arm_math.h"
#include "includes.h"

#define Rocker_Sensitivity  0.3f
pos_t target_pos[5]={0.0f};
line_t Lin[4]={0.0f};
u8 operation_step=0;
u8 control_right=0;  //机器人控制权（0手动，1自动）

void CM_task(void *p_arg){	
	OS_ERR err;
	short vx,vy,vz;
	short velo[4]={0};   //底盘四轮转速
	float temp1=10,temp2,temp3;   //缓冲
	p_arg = p_arg;
	
	while(1){
		if(Tele_Receiv_error<10){  //10次之内的错误认为正常扰动，视为遥控连接正常
			Tele_Receiv_error++;        //悲观地认为它就是不正常的，记录一次
			if(TelCtrlData.switch_l==1)    //自动控制
			{
				control_right=1;//默认自动控制
				switch(operation_step)
				{
					case 0:{ 						//从启动区定位到球架
							if(pos_y-O_y<0.65f*target_pos[0].y)
							{
								if(temp1<300)
									temp1+=5;
							}
							else
							{
								if(temp1>70)
									temp1-=10;
					  		open2_flag=3;
							}
							temp2=-10;
							temp3=(45+zangle)*0.0174533f;
							
							if((fmessage&0x03)!=0x00) operation_step=2;
							else if(pos_y-O_y>target_pos[0].y+200) operation_step++;
							
							vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
							vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));								
							vz=0.005f*(zangle-target_pos[0].angle)*400;							
					}break;
					case 1:{						//手动对球
							control_right=0;
							open2_flag=3;
							vy=0.5f*(TelCtrlData.left_y-1024);
							vx=Rocker_Sensitivity*(TelCtrlData.left_x-1024);
							vz=0.02f*(zangle-target_pos[0].angle)*400+0.2f*(TelCtrlData.right_x-1024);	
							if((fmessage&0x03)!=0x00) operation_step++;
							else if(TelCtrlData.switch_r==1)  operation_step++;
					}break;
					case 2:{            //夹球，后撤，定点
								open2_flag=0;			
								if(P_PosYforStart(pos_y-O_y,target_pos[0].y-1000,&temp1)) operation_step++;			
								temp2=P_PosXforStart(pos_x-O_x,Lin[0].xst);
								temp3=(45+zangle)*0.0174533f;
							
								if(pos_y-O_y<target_pos[0].y-200)
									vz=0.008f*(Lin[0].angle-135+zangle)*400;	
								else
								{
									temp2=0;
									vz=0.02f*(zangle-target_pos[0].angle)*400;
								}
								vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
								vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));	
					}break;
					case 3:{            //球架到交接区	
							if(!Get_Velt(&vx,&vy,&vz,0))  
							{
								CB_UP();
								temp1=PD_PosYforStop(pos_y-O_y,target_pos[1].y);
								temp2=-Rocker_Sensitivity*(TelCtrlData.left_x-1024);
							//	temp2=PD_PosXforStop(pos_x-O_x,target_pos[1].x);								
								temp3=(45+zangle)*0.0174533f;
							
								if((pos_y-O_y-target_pos[1].y)<=50&&(pos_y-O_y-target_pos[1].y)>=-50)
								{
									temp1=0;
									if(((zangle-target_pos[1].angle)<4)&&((zangle-target_pos[1].angle)>-4))
										operation_step++;
								}
								vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
								vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));						
								vz=0.01f*(zangle-target_pos[1].angle)*400;
							}
					}break;
					case 4:{            //交球
							control_right=0;
							vy=0.5f*(TelCtrlData.left_y-1024);
							vx=0.2f*(TelCtrlData.left_x-1024);
							vz=0.03f*(TelCtrlData.right_x-1024);	
							if(open2_flag>=2)
								if(TelCtrlData.switch_r==2) {CB_DOWN();operation_step++;}
					}break;
					case 5:{            //回球架区	
						if(P_PosYforStart(pos_y-O_y,target_pos[1].y+1000,&temp1)) operation_step++;
						temp2=P_PosXforStart(pos_x-O_x,Lin[1].xst);
						temp3=(45+zangle)*0.0174533f;
					
						vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
						vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));						
						vz=0.005f*(Lin[1].angle+45+zangle)*400;  //（Lin[1].angle+180）-45+zangle
					}break;
					case 6:{	
							if(!Get_Velt(&vx,&vy,&vz,1))   //获取速度与定位状态,到达目标位置后转交手动
							{
								
								temp1=PD_PosYforStop(pos_y-O_y,target_pos[2].y);
								temp2=-30;//Rocker_Sensitivity*(TelCtrlData.left_x-1024);								
								temp3=(45+zangle)*0.0174533f;
							
								if(temp1<70) temp1=70;	
								if((fmessage&0x03)!=0x00){
									open2_flag=3;
									temp1=0;
									CB_UP();
									if(TelCtrlData.switch_r==1)	{open2_flag=0;CB_DOWN();operation_step=8;}
								}
							//	else if(pos_y-O_y>target_pos[2].y+200)		operation_step++;
								vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
								vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));						
								vz=0.01f*(zangle-target_pos[2].angle)*400;
							}
					}break;
					case 7:{            //取球
							control_right=0;
							open2_flag=2;
							if(gold_num>0)
								CB_UP();
							vy=0.5f*(TelCtrlData.left_y-1024);
							vx=Rocker_Sensitivity*(TelCtrlData.left_x-1024);
							vz=0.02f*(zangle-target_pos[0].angle)*400+0.2f*(TelCtrlData.right_x-1024);
							if((fmessage&0x03)!=0x00) {CB_DOWN();operation_step++;}
							else if(TelCtrlData.switch_r==1) {CB_DOWN();operation_step++;}
					}break;
					case 8:{            //发车
							open2_flag=0;
							if(P_PosYforStart(pos_y-O_y,target_pos[2].y-1000,&temp1)) operation_step++;
							temp2=P_PosXforStart(pos_x-O_x,Lin[2].xst);
							temp3=(45+zangle)*0.0174533f;
						
							if(pos_y-O_y<target_pos[2].y-200)
								vz=0.008f*(Lin[2].angle-135+zangle)*400;	
							else
							{
								temp2=0;
								vz=0.02f*(zangle-target_pos[2].angle)*400;
							}
							vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
							vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));
																	
					}break;
					case 9:{            //停车
							open2_flag=0;					
							if(!Get_Velt(&vx,&vy,&vz,2))  
							{	
								temp1=PD_PosYforStop(pos_y-O_y,target_pos[3].y);
								temp2=-Rocker_Sensitivity*(TelCtrlData.left_x-1024);
								temp3=(45+zangle)*0.0174533f;
								
								vz=0.01f*(zangle-target_pos[3].angle)*400;	
								if(vz>50) vz=50;
								else if(vz<-50) vz=-50;
								
								if((pos_y-O_y-target_pos[3].y)<=50&&(pos_y-O_y-target_pos[3].y)>=-50)
								{
									temp1=0;
									if(vz<8&&vz>-8) operation_step++;
								}
								vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
								vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));	
							}
					}break;
					case 10:{           //交球
							control_right=0;
							CB_UP();
							vy=0.5f*(TelCtrlData.left_y-1024);
							vx=0.2f*(TelCtrlData.left_x-1024);
							vz=0.03f*(TelCtrlData.right_x-1024);
							if(gold_num<=1){
								gold_num=0;
								if(open2_flag>=3)
									if(TelCtrlData.switch_r==3) {CB_DOWN();open2_flag=3;operation_step++;}
							}
							else{
								if(open2_flag>=1)
									if(TelCtrlData.switch_r==3) 
									{
										CB_DOWN();
										delay_ms(500);
										gold_num-=2;
										open2_flag=2;
										operation_step++;
									}
							}
					}break;
				/*************刷分去咯************/
					case 11:{           //回球架区
						if(P_PosYforStart(pos_y-O_y,target_pos[3].y+1000,&temp1)) operation_step++;
						temp2=P_PosXforStart(pos_x-O_x,Lin[3].xst);
						temp3=(45+zangle)*0.0174533f;
					
						vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
						vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));								
						vz=0.005f*(Lin[3].angle+45+zangle)*400;  //（Lin[1].angle+180）-45+zangle
					}break;
					case 12:{           //停靠取球
							if(!Get_Velt(&vx,&vy,&vz,3)) 
							{					
								temp1=PD_PosYforStop(pos_y-O_y,target_pos[4].y);
								temp2=Rocker_Sensitivity*(TelCtrlData.left_x-1024);
								temp3=(45+zangle)*0.0174533f;
							
								if(pos_y-O_y>target_pos[4].y-400)	operation_step=7;
								vy=(short)(temp1*arm_sin_f32(temp3)+temp2*arm_cos_f32(temp3));
								vx=(short)(-temp1*arm_cos_f32(temp3)+temp2*arm_sin_f32(temp3));							
								vz=0.01f*(zangle-target_pos[4].angle)*400;								
							}
					}break;	
					default:{
						control_right=0;
						vy=TelCtrlData.left_y-1024;
						vx=TelCtrlData.left_x-1024;
						vz=0.5*(TelCtrlData.right_x-1024);	
					}
				}		
			}
			else{                    //手动控制
				control_right=0;
				vy=(TelCtrlData.left_y-1024);
				vx=0.5f*(TelCtrlData.left_x-1024);
				vz=0.1f*(TelCtrlData.right_x-1024);	
			}
			
			if(control_right==1){   //速度合成
				velo[0]=0.5f*(vy+vz);       	
				velo[1]=0.5f*(-vx-vz);
				velo[2]=0.5f*(vy-vz);   
				velo[3]=0.5f*(-vx+vz);
			}
			else{
				velo[0]=0.2f*(vy+vx+vz);  //vy     	//速度合成
				velo[1]=0.2f*(vy-vx-vz);
				velo[2]=0.2f*(vy+vx-vz);   //vy
				velo[3]=0.2f*(vy-vx+vz);
			}
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel1_ID,5000,velo[0]);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel2_ID,5000,velo[1]);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel3_ID,5000,velo[2]);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel4_ID,5000,velo[3]);
		}
		else  //我们已经给了它10次机会，So，遥控器连接异常，自动停车
		{   
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel1_ID,5000,0);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel2_ID,5000,0);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel3_ID,5000,0);
			CAN_RoboModule_DRV_Velocity_Mode(0x00,Wheel4_ID,5000,0);
		}
		OSTimeDly(13,OS_OPT_TIME_PERIODIC,&err); //延时13ms	
	}
}
  
u8 Get_Velt(short *v_x,short *v_y,short *v_z,u8 path)
{
	static float this_y[2]={0.0};
	short accel=0;     //定义加速度

	/***********计算Vz*************/
	if(path==0||path==2)
		*v_z = 0.15f*(Lin[path].angle-135+zangle)*400;     //d(v_z)=k*wL/2=k*w*400
	else 
		*v_z = 0.08f*(Lin[path].angle+45+zangle)*400;
	
	if(*v_z>50)
		*v_z=50;
	else if(*v_z<-50)
		*v_z=-50;
	
	/************计算Vy和Vx************/	
	this_y[1]=pos_y-O_y;
	if(path==0||path==2)
	{
		if(this_y[1]<=target_pos[path+1].y+2000)    //距离目标2.5m开始刹车
		{
				accel=-120;
				if((this_y[1]<=target_pos[path+1].y+1000)||*v_x<=130)   //距离目标1m,开始精准定位
					return 0;
		}
		else
		{
				if(*v_x<=880)     //前进速度未加到最大
					accel=120;
				else
					accel=0;
		}
		*v_x+=accel;
		*v_y=-0.9f*(TelCtrlData.left_x-1024);//-0.8f*temp;
		if(*v_y > 300)  *v_y = 300;
		else if(*v_y < -300) *v_y = -300;
		return 1;
	}
	else 
	{
		if(this_y[1]>=target_pos[path+1].y-2300)    //距离目标4m开始刹车
		{
				accel=120;
				if((this_y[1]>=target_pos[path+1].y-1000)||*v_x>=-130)   //距离目标1m,开始精准定位
					return 0;
		}
		else
	  {
				if(*v_x>=-880)     //前进速度未加到最大
					accel=-120;
				else
					accel=0;
  	}
		*v_x+=accel;
		*v_y=0.9f*(TelCtrlData.left_x-1024);//-0.8f*temp;
		if(*v_y > 300)  *v_y = 300;
		else if(*v_y < -300) *v_y = -300;
		return 1;
	}
}

short PD_PosYforStop(float this_pos,float target_pos)
{  
	const float k_p=0.8;
	const float k_d=-5;
	
	static float error[2] = {0.0,0.0};
	static float speed=0;
			
	error[0] = error[1];
	error[1] = -this_pos+target_pos;
	
	speed = error[1] * k_p + (error[1] - error[0]) * k_d; 
	
	if(speed > 80)  speed = 80;
	else if(speed < -80) speed = -80;
	
	return speed;
}

short PD_PosXforStop(float this_pos,float target_pos)
{
	const float k_p=0.8;
	const float k_d=0;
	
	static float error[2] = {0.0,0.0};
	static float speed=0;
	
	error[0] = error[1];
	error[1] = -this_pos+target_pos;
	
	speed = error[1] * k_p + (error[1]-error[0])*k_d; 

	if(speed > 240)  speed = 240;
	else if(speed < -240) speed = -240;
	
	return speed;
}

u8 P_PosYforStart(float this_pos,float target_pos,float *v_y)
{
	const float k_p=0.16;	
	static float offset=0;	
  float error=0;

	if(offset==0)
		offset=	k_p*(-this_pos+target_pos);   //自校准
	
	error = -this_pos+target_pos;
	*v_y=-(k_p*error-offset);
	if(error>0)
		*v_y+=100;
	else
		*v_y-=100;
	
	if(error<50&&error>-50)
	{
		offset=0;
		return 1;
	}
	return 0;
}

float P_PosXforStart(float this_pos,float target_pos)
{
	const float k_p=0.5;
	
	static float error[2] = {0.0,0.0};
	static float speed=0;
	
	error[0] = error[1];
	error[1] = -this_pos+target_pos;
	
	speed = k_p*error[1]; 

	if(speed > 64)  speed =64;
	else if(speed < -64) speed = -64;
	
	return speed;
}

short Pid_Velo_CCDX(short ccd_pos,short target_pos,short vy)
{
	const float P_p = 1.5; //2
	const float P_i = 0.002; //0.001
	const float P_d = 0;//1	
																										
	static float error_p[2] = {0.0,0.0};
	static float output = 0;
	static float error_p_sum= 0;
	
	if(ccd_pos==0||Crossed_flag==1)
		return 0;
	
	if(ccd_pos>150)
		ccd_pos-=256;
	
	error_p[0] = error_p[1];
	error_p[1] = -ccd_pos + target_pos;
	error_p_sum += error_p[1];
	
	if(error_p_sum>40000) 
		error_p_sum=40000;
	else if(error_p_sum<-40000) 
		error_p_sum=-40000;
	
	output =  error_p[1]  * P_p* (0.0015*vy+1)
				 +  error_p_sum * P_i 
				 + (error_p[1] - error_p[0]) * P_d;

	if(output > 400)  output =  400;
	if(output < -400) output = -400;

	return output;
}

