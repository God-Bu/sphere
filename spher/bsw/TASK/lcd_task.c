#include "lcd.h"
#include "task.h"
#include "can1.h"
#include "gpio.h"
#include "math.h"
#include "delay.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "uart6.h"
#include "24cxx.h"
#include "stdio.h"
#include "string.h"
#include "includes.h"

void LCD_task(void *p_arg){
	OS_ERR err;
	u8 count=0;
	u8 show_step=1,choice_twicel=0,choice_twicer=0,resume_flag=0; //二次确认，防止误触
	int show_newstep=0;
	char buff0[5],buff1[10],buff2[10],buff3[10];
	float temp_x[6],temp_y[6],temp_z[6];
	double temp1,temp2;
	union{ 
		u8 cdata[129];
		float fdata[32];
  }eeprom_data;
	
	p_arg = p_arg;
	
	clear_screen();
	display_GB2312_string(4,24,(u8*)" Hello,DMX"); //一句问候	
	delay_ms(2000);
	clear_screen();
	while(1){
		if((show_step!=show_newstep)||resume_flag)         //显示状态信息，状态变化时再更新
		{
			resume_flag=0;
			show_step=show_newstep;
			clear_screen();
			switch(show_step)
			{
				case 0:	
					display_GB2312_string(2,5,(u8*)" 要重新定点吗？");
					display_GB2312_string(5,20,(u8*)"YES");
					display_GB2312_string(5,90,(u8*)"NO");
					break;
				case 1:            //标定原点
					display_GB2312_string(2,25,(u8*)" 起点坐标");
					break;
				case 2:case 3:case 4:        //采集各点坐标
					sprintf(buff0,"%d",(show_step-1));
					display_GB2312_string(2,30,(u8*)" 第   点");
					display_GB2312_string(2,62,(u8*)buff0);
					break;
				case 5:            //计算定位参数
					display_GB2312_string(4,1,(u8*)" 数据处理中。。。");
					break;
				case 6:            //写入EEPROM
					display_GB2312_string(4,1,(u8*)" 数据写入中。。。");
					break;
				case 7:            //读取EEPROM
					display_GB2312_string(4,1,(u8*)" 数据读取中。。。");
					break;
				case 8:          
					display_GB2312_string(4,10,(u8*)" OK！正在载入");
				default:
					break;
			}
		}
		else if(choice_twicel)
		{
			display_GB2312_string(2,21,(u8*)" 确认选择？");
			display_GB2312_string(5,30,(u8*)"NO");
			display_GB2312_string(5,80,(u8*)"YES");
		}
		else if(choice_twicer)
		{
			display_GB2312_string(2,21,(u8*)" 确认取消？");
			display_GB2312_string(5,20,(u8*)"YES");
			display_GB2312_string(5,90,(u8*)"NO");
		}

		if((show_step>0&&show_step<5)&&(choice_twicel||choice_twicer)==0)             //显示坐标信息，实时更新
		{
			clear_screen_half();
			sprintf(buff1,"%.1f",(double)pos_x);
			sprintf(buff2,"%.1f",(double)pos_y);
			sprintf(buff3,"%.1f",(double)zangle);
			
			display_string_5x8(5,5,(u8*)"Pos(");
			display_string_5x8(5,29,(u8*)buff1);
			display_string_5x8(5,71,(u8*)",");
			display_string_5x8(5,77,(u8*)buff2);
			display_string_5x8(5,119,(u8*)")");
			display_string_5x8(7,5,(u8*)"Angle:");
			display_string_5x8(7,41,(u8*)buff3);
		}
		
		if(show_step==8)           //结束此任务
		{
#if EN_CM_TASK
			//定位模块校准
			while(Action_calibtation())
			{
				delay_us(5000);
			}
#endif
#if EN_MOTOR_INIT>0u      //配置所有电机
			CAN_RoboModule_DRV_Reset(0x00,0x00);
			delay_ms(500);
			CAN_RoboModule_DRV_Mode_Choice(0x00,0x01,Velocity_Mode);	
			CAN_RoboModule_DRV_Mode_Choice(0x00,0x02,Velocity_Mode);	
			CAN_RoboModule_DRV_Mode_Choice(0x00,0x03,Velocity_Mode);	
			CAN_RoboModule_DRV_Mode_Choice(0x00,0x04,Velocity_Mode);	
			delay_ms(500);
			CAN_RoboModule_DRV_Config(0x00,0x00,0x00,0x00);              
			delay_ms(500);
#endif
			//与从机通信
		USART_SendData(USART1,CCD_NoSend);                 //CCD不发送

		USART_SendData(USART2,Front_Send);                 //激光雷达开机
		delay_ms(500);
		USART_SendData(USART2,Front_ON);   
//		delay_ms(500);
//		USART_SendData(USART2,Front_Pick);   //向前驱索要数据						
//	
		//初始化结束提醒
			CR_OPEN();
			delay_ms(150);
			CM_OPEN();
			delay_ms(150);
			CL_OPEN();
			lcd_led()=0;            //关背光灯
			Transfer_command(0xae);  //关显示
			OS_TaskResume((OS_TCB*)&TELETaskTCB,&err);
			OS_TaskResume((OS_TCB*)&CMTaskTCB,&err);
			OS_TaskResume((OS_TCB*)&CylinderTaskTCB,&err);
			OS_TaskSuspend((OS_TCB*)&LCDTaskTCB,&err);		//挂起LCD任务
		}
		if(show_step==7)            //读数据
		{
			AT24CXX_Read(0,(u8*)eeprom_data.cdata,129);
			if(eeprom_data.cdata[0]==0x11)
			{
				if(eeprom_data.cdata[128]==0x05)
				{
					for(count=0;count<5;count++)        //读坐标
					{
						target_pos[count].x=eeprom_data.fdata[count*3+1];      //1+count*3+0
						target_pos[count].y=eeprom_data.fdata[count*3+2];
						target_pos[count].angle=eeprom_data.fdata[count*3+3];
					}
					for(count=0;count<4;count++)       //读直线特征
					{
						Lin[count].angle=eeprom_data.fdata[count*4+16];      
						Lin[count].k=eeprom_data.fdata[count*4+17];
						Lin[count].b=eeprom_data.fdata[count*4+18];
						Lin[count].xst=eeprom_data.fdata[count*4+19];	
					}	
					show_newstep++;
					clear_screen();
				}	
				else
					display_GB2312_string(1,14,(u8*)" Read ERROR!");
			}
			else
				display_GB2312_string(1,14,(u8*)" Read ERROR!");
		}
		else if(show_step==6)            //存数据
		{
			AT24CXX_Write(0,(u8*)eeprom_data.cdata,129);
			show_newstep++;
			clear_screen();
		}
		else if(show_step==5)            //数据运算
		{
			eeprom_data.cdata[0]=0x11;
			for(count=0;count<5;count++)
			{
				eeprom_data.fdata[count*3+1]=temp_x[count+1]-temp_x[0];
				eeprom_data.fdata[count*3+2]=temp_y[count+1]-temp_y[0];
				eeprom_data.fdata[count*3+3]=temp_z[count+1];
			}
			for(count=0;count<4;count++)
			{
				temp1=temp_x[count+2]-temp_x[count+1];
				temp2=temp_y[count+2]-temp_y[count+1];
				eeprom_data.fdata[count*4+16]=(float)(atan2(temp1,temp2)*180/3.1415926f);   //A，15+(count-5)*4+1
				
				temp1=temp1/temp2;
				eeprom_data.fdata[count*4+17]=(float)temp1;   //K
				
				temp2=temp_x[count+1]-temp1*temp_y[count+1];
				eeprom_data.fdata[count*4+18]=(float)temp2;   //B
				
				if(count==1||count==3)
					eeprom_data.fdata[count*4+19]=(float)(temp_x[count+1]+1000*temp1);   //F
				else   //count==0||count==2
					eeprom_data.fdata[count*4+19]=(float)(temp_x[count+1]-1000*temp1);   //F
			}
			eeprom_data.cdata[128]=0x05;
			show_newstep++;
			clear_screen();
		}
		
		if(Button_Left()==1)       //读取按键
		{
			delay_ms(80);           //按键消抖
			if(Button_Left()==1)
			{
				if(show_newstep<5)     //后面几种显示状态由系统制动更改
				{
					clear_screen();
					if(choice_twicer==1)
					{
						choice_twicer=0;     //二次确认
						show_newstep--;
						if(show_newstep<0)
							show_newstep=7;
					}
					else if(choice_twicel==0)
						choice_twicel=1;
					else if(choice_twicel==1)
					{
						choice_twicel=0;
						resume_flag=1;
					}
				}
			}
		}
		else if(Button_Right()==1)
		{
			delay_ms(80);
			if(Button_Right()==1)
			{
				if(show_newstep<5)     //后面几种显示状态由系统制动更改
				{
					clear_screen();
					if(choice_twicel==1)
					{
						choice_twicel=0;     //二次确认
						if(show_step>0)       //记录位置信息
						{
							switch(show_step)
							{
								case 1:
									temp_x[0]=pos_x;
									temp_y[0]=pos_y;
									temp_z[0]=zangle;
									break;
								case 2:
									temp_x[1]=temp_x[3]=temp_x[5]=pos_x;
									temp_y[1]=temp_y[3]=temp_y[5]=pos_y;
									temp_z[1]=temp_z[3]=temp_z[5]=zangle;
									break;
								case 3:
									temp_x[2]=pos_x;
									temp_y[2]=pos_y;
									temp_z[2]=zangle;
									break;
								case 4:
									temp_x[4]=pos_x;
									temp_y[4]=pos_y;
									temp_z[4]=zangle;
									break;
							}
						}
						show_newstep++;
					}
					else if(choice_twicer==0)
						choice_twicer=1;
					else if(choice_twicer==1)
					{
						choice_twicer=0;
						resume_flag=1;
					}
				}
			}
			
		}	
		OSTimeDly(50,OS_OPT_TIME_PERIODIC,&err); //延时50ms	
	}
}

