#include "uart6.h"
#include "tim2.h"
#include "includes.h"					//ucos 使用	  

uint8_t sbus_rx_buffer[35];//数组

Acc Acc_Data;
Gyrc Gyrc_Data;
Angle Angle_Data;
Acc_angle Acc_angle_Data;
#define g 9.8;
#define mid_Z_angle 45;

double angle_yaw=0;                 //消除极值点 让角度变换变得连续 
int16_t round_count=0;
double Yaw_angle[2];

//Tel_Ctrl_t TelCtrlData;

float time;
float V_angle[2];
float V_test;

void USART6_Config(void){
    USART_InitTypeDef usart6;
    GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2,ENABLE);//DMA双AHB主总线构架，一个用于存储器访问，一个用于外设访问
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6 ,GPIO_AF_USART6);//复用引脚
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7 ,GPIO_AF_USART6);//复用引脚

    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 ;//配置引脚
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC,&gpio);

	nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    USART_DeInit(USART6);
    usart6.USART_BaudRate = 115200;                //SBUS 100K baudrate
    usart6.USART_WordLength = USART_WordLength_8b;//字长8B
    usart6.USART_StopBits = USART_StopBits_1;//一个停止位
    usart6.USART_Parity = USART_Parity_No;//奇偶校验位
    usart6.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//串口模式  接受模式
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6,&usart6);
	
    DMA_DeInit(DMA2_Stream2);
    dma.DMA_Channel= DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//外设数据寄存器地址
    dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;//内存地址
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//数据传输方向  外设到内存
    dma.DMA_BufferSize = 35;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不自增
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存器地址自增
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;//循环采集
    dma.DMA_Priority = DMA_Priority_VeryHigh;//DMA通道优先级
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//先入先出（FIFO）
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//选择FIFO阈值
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发传输配置
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发传输配置
    DMA_Init(DMA2_Stream2,&dma);
    DMA_Cmd(DMA2_Stream2,ENABLE);
		
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);//开启串口DMA接收功能
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	USART_Cmd(USART6,ENABLE);
	DMA_Cmd(DMA2_Stream2,ENABLE);
}



void USART6_IRQHandler(void)
{
	u8 Current_NDTR=0;
	OSIntEnter();
	
	if (USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)  
   {  
		 //清除中断标志    
		 (void)USART6->SR;   
		 (void)USART6->DR;
		 
		 if(DMA2->LISR&DMA_IT_TCIF2)        //DMA传输完成
		 {
			 DMA2->LIFCR=DMA_FLAG_TCIF2&(uint32_t)0x0F7D0F7D;     //清除传输完成标志位			 
			 DMA_Cmd(DMA2_Stream2, DISABLE);                      //关闭DMA  
			 Current_NDTR=DMA2_Stream2->NDTR;                     //读取剩余缓存量
			 DMA2_Stream2->NDTR=35;                               //重载DMA缓冲值  
			 DMA_Cmd(DMA2_Stream2, ENABLE);                       //重启DMA	
		 
			 if(Current_NDTR==0x02)       //若剩余2个缓冲字节，说明这一帧数据完整
			 {
				 Data_Process(sbus_rx_buffer);
			 }			
		 }		 		 
	 }
	 OSIntExit(); 
}


void Data_Process(uint8_t *sbus_rx_buffer)
{
	static uint8_t check_sum[3]={0,0,0};
	static float time_gap;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<10;j++)
		{
			check_sum[i]+=sbus_rx_buffer[j+i*11];
		}
	}
	if(check_sum[0]==sbus_rx_buffer[10] && check_sum[1]==sbus_rx_buffer[21] && check_sum[2]==sbus_rx_buffer[32])             //数据正确
	{
		if(sbus_rx_buffer[0]==0x55 && sbus_rx_buffer[1]==0x51)          //解析加速度
		{
			Acc_Data.Acc_x=(float)((short)(( (short)sbus_rx_buffer[3]<<8) | sbus_rx_buffer[2]))/32768*16*g;
			Acc_Data.Acc_y=(float)((short)(( (short)sbus_rx_buffer[5]<<8) | sbus_rx_buffer[4]))/32768*16*g;
			Acc_Data.Acc_z=(float)((short)(( (short)sbus_rx_buffer[7]<<8) | sbus_rx_buffer[6]))/32768*16*g;
			Acc_Data.T=(float)((short)(( (short)sbus_rx_buffer[9]<<8) | sbus_rx_buffer[10]))/100;
		}
		if(sbus_rx_buffer[11]==0x55 && sbus_rx_buffer[12]==0x52)          //解析角速度
		{
			Gyrc_Data.Gyrc_x=(float)((short)(( (short)sbus_rx_buffer[14]<<8) | sbus_rx_buffer[13]))/32768*2000;
			Gyrc_Data.Gyrc_y=(float)((short)(( (short)sbus_rx_buffer[16]<<8) | sbus_rx_buffer[15]))/32768*2000;
			Gyrc_Data.Gyrc_z=(float)((short)(( (short)sbus_rx_buffer[18]<<8) | sbus_rx_buffer[17]))/32768*2000;
			Gyrc_Data.T=(float)((short)(( (short)sbus_rx_buffer[20]<<8) | sbus_rx_buffer[19]))/100;
		}
		if(sbus_rx_buffer[22]==0x55 && sbus_rx_buffer[23]==0x53)          //解析角度
		{
			Angle_Data.Angle_x=(float)((short)(( (short)sbus_rx_buffer[25]<<8) | sbus_rx_buffer[24]))/32768*180;
			Angle_Data.Angle_y=(float)((short)(( (short)sbus_rx_buffer[27]<<8) | sbus_rx_buffer[26]))/32768*180;
			Angle_Data.Angle_z=(float)((short)(( (short)sbus_rx_buffer[29]<<8) | sbus_rx_buffer[28]))/32768*180;
			Angle_Data.Angle_z=Angle_Data.Angle_z-mid_Z_angle;
			Angle_Data.T=(float)((short)(( (short)sbus_rx_buffer[31]<<8) | sbus_rx_buffer[30]))/100;
		}
	}
	
	
//	time=time_count*0.1+TIM_GetCounter(TIM2)*0.00001;
//	V_angle[0]=V_angle[1];
//	V_angle[1]=Gyrc_Data.Gyrc_x;
//	V_test=V_angle[1]-V_angle[0];
	time_gap=time_count*0.1+TIM_GetCounter(TIM2)*0.00001;
	//time=time_gap;
	Acc_angle_Data.Acc_angle_x=Acc_angle_produce_x(Gyrc_Data.Gyrc_x,time_gap);
	Acc_angle_Data.Acc_angle_y=Acc_angle_produce_x(Gyrc_Data.Gyrc_y,time_gap);
	Acc_angle_Data.Acc_angle_z=Acc_angle_produce_x(Gyrc_Data.Gyrc_z,time_gap);
	time_count=0;
	TIM2->CNT=0;                                                                                                  
	
	Yaw_angle[0]=Yaw_angle[1];                             //消除极值点
	Yaw_angle[1]=Angle_Data.Angle_z;
	if(Yaw_angle[1]-Yaw_angle[0]>300)round_count--;
	if(Yaw_angle[1]-Yaw_angle[0]<-300)round_count++;
	angle_yaw=round_count*360.0f+Yaw_angle[1];
	//Angle_Data.Angle_y=Angle_Y_Process(Gyrc_Data.Gyrc_y, Angle_Data.Angle_y);
	check_sum[0]=check_sum[1]=check_sum[2]=0;               //校验值清零
}


float Angle_Y_Process(float Gyrc, float Angle)           //Y轴角度 -90 -- +90
{
	static float Gyrc_y[2] = {0.0,0.0};             //记录上一刻的角速度和角度
	static float Angle_y[2] = {0.0,0.0};
	static float return_value[2] = {0.0,0.0};
	
	Gyrc_y[0]=Gyrc_y[1];
	Gyrc_y[1]=Gyrc;
	Angle_y[0]=Angle_y[1];
	Angle_y[1]=Angle;
	return_value[0]=return_value[1];
	
	if(Gyrc_y[1]-Gyrc_y[0]>0 && Angle_y[1]-Angle_y[0]>0)          //第一二象限正转
	{
		return_value[1]=Angle_y[1];                                //正常情况       
		return return_value[1];                                       
	}
	else if(Gyrc_y[1]-Gyrc_y[0]>0 && Angle_y[1]-Angle_y[0]<0)     //三四象限正转
	{
		if(Angle_y[1]>=0)
		{
			return_value[1]=180-Angle_y[1];                         //第三象限
			return return_value[1];                                
		}			
		else
		{
			return_value[1]=-180-Angle_y[1];                        //第四象限                        
			return return_value[1];                             
		}			
	}
	else if(Gyrc_y[1]-Gyrc_y[0]<0 && Angle_y[1]-Angle_y[0]<0)        //第一二象限反转
	{
		return_value[1]=Angle_y[1];                                //正常情况       
		return return_value[1]; 
	}
	else if(Gyrc_y[1]-Gyrc_y[0]<0 && Angle_y[1]-Angle_y[0]>0)        //三四象限反转
	{
		if(Angle_y[1]>=0)
		{
			return_value[1]=180-Angle_y[1];                         //第三象限
			return return_value[1];                                
		}			
		else
		{
			return_value[1]=-180-Angle_y[1];                        //第四象限                        
			return return_value[1];                             
		}
	}
	else                                                           //极少出现
	{
		return return_value[1];
	}
	
}




float Acc_angle_produce_x(float Gyrc_x,float time_count)
{
	static float Gyrc_raw_data_x[2]={0.0,0.0};
	
	Gyrc_raw_data_x[0]=Gyrc_raw_data_x[1];
	Gyrc_raw_data_x[1]=Gyrc_x;
	
	return (Gyrc_raw_data_x[1]-Gyrc_raw_data_x[0])/time_count;
}

float Acc_angle_produce_y(float Gyrc_y,float time_count)
{
	static float Gyrc_raw_data_y[2]={0.0,0.0};
	
	Gyrc_raw_data_y[0]=Gyrc_raw_data_y[1];
	Gyrc_raw_data_y[1]=Gyrc_y;
	
	return (Gyrc_raw_data_y[1]-Gyrc_raw_data_y[0])/time_count;
}

float Acc_angle_produce_z(float Gyrc_z,float time_count)
{
	static float Gyrc_raw_data_z[2]={0.0,0.0};
	
	Gyrc_raw_data_z[0]=Gyrc_raw_data_z[1];
	Gyrc_raw_data_z[1]=Gyrc_z;
	
	return (Gyrc_raw_data_z[1]-Gyrc_raw_data_z[0])/time_count;
}




/*******************************************************************************/	
/*	HEX		      F				|		    F       |        F         |        F          */
/*                      |               |                  |                   */
/*  BIN		1		1		1		1	|	1		1		1		1	|	1		1		 1		1	 |	1		1		1		1    */
/*											|               |                  |                   */
/*  Key		B		V		C		X |	Z		G		F		R	|	E		Q	 Ctrl Shift|	D		A		S		W    */
/*******************************************************************************/
