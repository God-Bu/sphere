#include "task.h"
#include "delay.h"
#include "ccd.h"
#include "includes.h"

#define T 108

u16 CCD_Pos=0;
u8 Pixel3[155]={0};
void CCD_Task(void *p_arg){
	OS_ERR err;
	u8 Pixel1[128]={0};
	u8 Pixel2[128]={0};
	u8 i,j;	
	u16 current_pos=0;
	p_arg = p_arg;
	
	while(1)
	{		
		//	OSSchedLock(&err);          //给调度器上锁，其它任务干扰
			StartIntegrate();
			ImageCapture(Pixel1,Pixel2);      	
			current_pos=0;
			for(i=18,j=0;j<82;j++,i++)
			{
				if(Pixel1[i]>=T)
				{	
					Pixel3[j]=1;
					current_pos+=j;
				}
				else
					Pixel3[j]=0;
			}
			for(i=18,j=82;j<154;j++,i++)
			{
				if(Pixel2[i]>=T)
				{
					current_pos+=j;
					Pixel3[j]=1;
				}
				else
					Pixel3[j]=0;
			}
			CCD_Pos=current_pos;
		//	OSSchedUnlock(&err);
		//	OSTimeDly(9,OS_OPT_TIME_PERIODIC,&err); //延时7ms	
	}
}

void ImageCapture(unsigned char * ImageData1,unsigned char * ImageData2)
{
	u8 i;
	TSL1401_SI1 = 1;				//SI便为高脉冲时间必须大于上次的采集之后20us
	TSL1401_SI2 = 1;
	delay_us(2);
	TSL1401_CLK1 = 1;
	TSL1401_CLK2 = 1;
	delay_us(2);
	TSL1401_SI1 = 0;
	TSL1401_SI2 = 0;
	delay_us(2);
  for(i=0;i<128;i++){
		* ImageData1=Get_Adc(PICTURE);
		ImageData1++;
		* ImageData2=Get_Adc(!PICTURE);
		ImageData2++;
		TSL1401_CLK1 = 0;
		TSL1401_CLK2 = 0;
		delay_us(4);
		TSL1401_CLK1 = 1;
		TSL1401_CLK2 = 1;
		delay_us(4);
	}
	TSL1401_CLK1 = 0;       //SI在CLK低电平时上升
	TSL1401_CLK2 = 0;
	delay_us(2);
}

void StartIntegrate(void)
{
  unsigned char i;
	TSL1401_SI1 = 1;	
	TSL1401_SI2 = 1;	
	delay_us(2);
	TSL1401_CLK1 = 1;
	TSL1401_CLK2 = 1;
	delay_us(2);
	TSL1401_SI1 = 0;
	TSL1401_SI2 = 0;
	delay_us(2);
	for(i=0;i<128;i++)
	{
		TSL1401_CLK1 = 0;
		TSL1401_CLK2 = 0;
		delay_us(4);
		TSL1401_CLK1 = 1; 
		TSL1401_CLK2 = 1;
		delay_us(4);
	}
	TSL1401_CLK1 = 0;       //SI在CLK低电平时上升
	TSL1401_CLK2 = 0;
	delay_us(2);
	delay_ms(3);
}
