#include "uart3.h"
#include "sys.h"
//#include "gpio.h"
//////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

uint8_t rx_buffer[22];

steering_engine_status Steer_engine_ID1;
steering_engine_status Steer_engine_ID2;
steering_engine_status Steer_engine_ID3;
steering_engine_status Steer_engine_ID4;

uint8_t buffer_test[2];
uint16_t test_aa;
int16_t test_bb;

void USART3_Init(void){
	USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef   dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 

	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	 
	gpio.GPIO_OType = GPIO_OType_PP; 
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&gpio); 

	nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    USART_DeInit(USART3);
    usart.USART_BaudRate = 1000000;                //UART3 115200bps
    usart.USART_WordLength = USART_WordLength_8b;//字长8B
    usart.USART_StopBits = USART_StopBits_1;//一个停止位
    usart.USART_Parity = USART_Parity_No;//奇偶校验位
    usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//串口模式  接受模式
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart);
	
	DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);//外设数据寄存器地址
    dma.DMA_Memory0BaseAddr = (uint32_t)rx_buffer;//内存地址
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//数据传输方向  外设到内存
    dma.DMA_BufferSize = 22;
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
    DMA_Init(DMA1_Stream1,&dma);
    DMA_Cmd(DMA1_Stream1,ENABLE);
	
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//开启串口DMA接收功能
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	USART_Cmd(USART3,ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);
	
}

void USART3_SendBuff(uint8_t *addr,uint8_t count_n)   //从一个地址开始发count_n次数据
{
	for(int i=0;i<count_n;i++)
	{
		USART_ClearFlag(USART3,USART_FLAG_TC);
		USART_SendData(USART3,*(addr++));
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//判断是否发送完成
	}
}

void get_steering_engine_status(void)   //读取ID1-ID4的舵机状态
{
	uint8_t send_buff[12];                 //查询舵机内存发送帧
	send_buff[1]=send_buff[0]=0xFF;        //帧头
	send_buff[2]=0xFE;                     //ID地址 广播ID
	send_buff[3]=0x08;                     //数据长度  参数长度+2
	send_buff[4]=0x82;                     //指令
	send_buff[5]=0x38;                     //参数1  读取数据的首地址
	send_buff[6]=0x0F;                     //参数2  读取数据的长度  
	send_buff[7]=0x01;                     //参数3  第一个舵机ID号
	send_buff[8]=0x02;                     //参数4  第二个舵机ID号
	send_buff[9]=0x03;                     //参数5  第三个舵机ID号
	send_buff[10]=0x04;                     //参数6  第三个舵机ID号
	send_buff[11]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+
					send_buff[6]+send_buff[7]+send_buff[8]+send_buff[9]+send_buff[10]);                     //校验位
	USART3_SendBuff(send_buff,12);
}


void set_steering_engine_velocity(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3)             //设置三个舵机的速度
{
	uint8_t send_buff[17];                  //控制舵机速度发送帧
	send_buff[1]=send_buff[0]=0xFF;         //帧头
	send_buff[2]=0xFE;                      //ID地址 广播ID
	send_buff[3]=0x0D;                      //数据长度  参数长度+2   11个参数
	send_buff[4]=0x83;                      //指令
	send_buff[5]=0x2E;                      //参数1  写数据的首地址
	send_buff[6]=0x02;                      //参数2  写取数据的长度  
	send_buff[7]=0x01;                      //参数3  第一个舵机ID号
	if(velocity_ID1>=0)
	{
		send_buff[8]=velocity_ID1;              //参数4  速度1的低8位
		send_buff[9]=velocity_ID1>>8;           //参数5  速度1的高8位
	}
	else                                       //不能用补码形式
	{
		send_buff[8]=-velocity_ID1;
		send_buff[9]=0x80;               //确认第一位方向
		send_buff[9]=send_buff[9]|(-velocity_ID1>>8);
	}
	send_buff[10]=0x02;                     //参数6  第二个舵机ID号
	if(velocity_ID2>=0)
	{
		send_buff[11]=velocity_ID2;             //参数7  速度2的低8位
		send_buff[12]=velocity_ID2>>8;          //参数8  速度2的高8位
	}
	else
	{
		send_buff[11]=-velocity_ID2;           
		send_buff[12]=0x80;
		send_buff[12]=send_buff[12]|(-velocity_ID2>>8);         
	}
	send_buff[13]=0x03;                     //参数9  第三个舵机ID号
	if(velocity_ID3>=0)
	{
		send_buff[14]=velocity_ID3;             //参数10  速度3的低8位
		send_buff[15]=velocity_ID3>>8;          //参数11  速度3的高8位
	}
	else
	{
		send_buff[14]=-velocity_ID3;       
		send_buff[15]=0x80;  
		send_buff[15]=send_buff[15]|(-velocity_ID3>>8);  
	}
	send_buff[16]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]);          //校验位
	USART3_SendBuff(send_buff,17);
}

void set_steering_engine_acceleration(uint8_t acceleration_ID1,uint8_t acceleration_ID2,uint8_t acceleration_ID3)             //设置三个舵机的加速度
{
	uint8_t send_buff[14];                  //控制舵机速度发送帧
	send_buff[1]=send_buff[0]=0xFF;         //帧头
	send_buff[2]=0xFE;                      //ID地址 广播ID
	send_buff[3]=0x0A;                      //数据长度  参数长度+2   11个参数
	send_buff[4]=0x83;                      //指令
	send_buff[5]=0x29;                      //参数1  写数据的首地址
	send_buff[6]=0x01;                      //参数2  写取数据的长度  
	send_buff[7]=0x01;                      //参数3  第一个舵机ID号
	send_buff[8]=acceleration_ID1;          //参数4  加速度1
	send_buff[9]=0x02;                      //参数5  第二个舵机ID号
	send_buff[10]=acceleration_ID2;         //参数6  加速度2
	send_buff[11]=0x03;                     //参数7  第三个舵机ID号
	send_buff[12]=acceleration_ID3;         //参数8  加速度3
	send_buff[13]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+
					send_buff[8]+send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]);          //校验位
	USART3_SendBuff(send_buff,14);
}



void USART3_IRQHandler(void) {  	  
	 u8 Current_NDTR=0; 
	 OSIntEnter();
	
	 if (USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)  
   {  
		 //清除中断标志    
		 (void)USART3->SR;   
		 (void)USART3->DR;
		 
		 if(DMA_GetFlagStatus(DMA1_Stream1,DMA_IT_TCIF1) == SET)        //DMA传输完成
		 {
			 DMA_ClearFlag(DMA1_Stream1, DMA_IT_TCIF1);         //清除传输完成标志位			 
			 DMA_Cmd(DMA1_Stream1, DISABLE);                      //关闭DMA  
			 Current_NDTR=DMA1_Stream1->NDTR;                     //读取剩余缓存量
			 DMA1_Stream1->NDTR=22;                               //重载DMA缓冲值  
			 DMA_Cmd(DMA1_Stream1, ENABLE);                       //重启DMA	
		 
			 if(Current_NDTR==0x01)       //若剩余1个缓冲字节，说明这一帧数据完整
			 {
				 steering_engine_Data_Process(rx_buffer);
			 }	 
			 
		 }		 		 
	 }
							
	 OSIntExit(); 
}


void steering_engine_Data_Process(uint8_t *rx_buffer)
{
	static uint8_t check_sum=0;
	if(rx_buffer[0]!=0xFF || rx_buffer[1]!=0xFF)return;           //帧头不对 
	for(int i=2;i<20;i++)
	{
		check_sum+=rx_buffer[i];
	}
	check_sum=~check_sum;
	if( check_sum==rx_buffer[20] )             //数据正确
	{
		switch(rx_buffer[2])                      //判断是哪个ID舵机反馈的消息
		{
			case 0x01:
			{
				Steer_engine_ID1.position = (rx_buffer[6]<<8) | rx_buffer[5];         //位置  4096(步)分辨率
				if((rx_buffer[8]>>7)==0)                    //正数
				{
					Steer_engine_ID1.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //速度  单位50步/s
				}
				else
				{
					Steer_engine_ID1.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID1.load = (rx_buffer[10]<<8) | rx_buffer[9];            //负载  驱动电机电压占空比 单位0.1%
				Steer_engine_ID1.Voltage = rx_buffer[11];                             //电压  单位0.1V
				Steer_engine_ID1.T = rx_buffer[12];                                   //温度  单位1摄氏度
				Steer_engine_ID1.flag_Awrite = rx_buffer[13];                         //异步写标识
				Steer_engine_ID1.flag_error = rx_buffer[14];                          //舵机错误标识  0表示无错误
				Steer_engine_ID1.flag_move = rx_buffer[15];                           //舵机移动标识  1表示运动 0表示停止
				Steer_engine_ID1.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //目标？
				Steer_engine_ID1.current = (rx_buffer[19]<<8) | rx_buffer[18];        //当前电流 单位6.5mA
			}break;
			case 0x02:
			{
				Steer_engine_ID2.position = (rx_buffer[6]<<8) | rx_buffer[5];         //位置  4096(步)分辨率
				if((rx_buffer[8]>>7)==0)                    //正数
				{
					Steer_engine_ID2.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //速度  单位50步/s
				}
				else
				{
					Steer_engine_ID2.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}    
				Steer_engine_ID2.load = (rx_buffer[10]<<8) | rx_buffer[9];            //负载  驱动电机电压占空比 单位0.1%
				Steer_engine_ID2.Voltage = rx_buffer[11];                             //电压  单位0.1V
				Steer_engine_ID2.T = rx_buffer[12];                                   //温度  单位1摄氏度
				Steer_engine_ID2.flag_Awrite = rx_buffer[13];                         //异步写标识
				Steer_engine_ID2.flag_error = rx_buffer[14];                          //舵机错误标识  0表示无错误
				Steer_engine_ID2.flag_move = rx_buffer[15];                           //舵机移动标识  1表示运动 0表示停止
				Steer_engine_ID2.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //目标？
				Steer_engine_ID2.current = (rx_buffer[19]<<8) | rx_buffer[18];        //当前电流 单位6.5mA
			}break;
			case 0x03:
			{
				Steer_engine_ID3.position = (rx_buffer[6]<<8) | rx_buffer[5];         //位置  4096(步)分辨率
				if((rx_buffer[8]>>7)==0)                    //正数
				{
					Steer_engine_ID3.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //速度  单位50步/s
				}
				else
				{
					Steer_engine_ID3.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID3.load = (rx_buffer[10]<<8) | rx_buffer[9];            //负载  驱动电机电压占空比 单位0.1%
				Steer_engine_ID3.Voltage = rx_buffer[11];                             //电压  单位0.1V
				Steer_engine_ID3.T = rx_buffer[12];                                   //温度  单位1摄氏度
				Steer_engine_ID3.flag_Awrite = rx_buffer[13];                         //异步写标识
				Steer_engine_ID3.flag_error = rx_buffer[14];                          //舵机错误标识  0表示无错误
				Steer_engine_ID3.flag_move = rx_buffer[15];                           //舵机移动标识  1表示运动 0表示停止
				Steer_engine_ID3.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //目标？
				Steer_engine_ID3.current = (rx_buffer[19]<<8) | rx_buffer[18];        //当前电流 单位6.5mA
			}break;
			case 0x04:
			{
				Steer_engine_ID4.position = (rx_buffer[6]<<8) | rx_buffer[5];         //位置  4096(步)分辨率
				if((rx_buffer[8]>>7)==0)                    //正数
				{
					Steer_engine_ID4.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //速度  单位50步/s
				}
				else
				{
					Steer_engine_ID4.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID4.load = (rx_buffer[10]<<8) | rx_buffer[9];            //负载  驱动电机电压占空比 单位0.1%
				Steer_engine_ID4.Voltage = rx_buffer[11];                             //电压  单位0.1V
				Steer_engine_ID4.T = rx_buffer[12];                                   //温度  单位1摄氏度
				Steer_engine_ID4.flag_Awrite = rx_buffer[13];                         //异步写标识
				Steer_engine_ID4.flag_error = rx_buffer[14];                          //舵机错误标识  0表示无错误
				Steer_engine_ID4.flag_move = rx_buffer[15];                           //舵机移动标识  1表示运动 0表示停止
				Steer_engine_ID4.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //目标？
				Steer_engine_ID4.current = (rx_buffer[19]<<8) | rx_buffer[18];        //当前电流 单位6.5mA
			}break;
			default: break;
		}
	}
	check_sum=0;               //校验值清零
}

void set_steering_engine_velocity_four(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3,int16_t velocity_ID4)             //设置三个舵机的速度
{
	uint8_t send_buff[20];                  //控制舵机速度发送帧
	send_buff[1]=send_buff[0]=0xFF;         //帧头
	send_buff[2]=0xFE;                      //ID地址 广播ID
	send_buff[3]=0x10;                      //数据长度  参数长度+2   16个参数
	send_buff[4]=0x83;                      //指令
	send_buff[5]=0x2E;                      //参数1  写数据的首地址
	send_buff[6]=0x02;                      //参数2  写取数据的长度  
	send_buff[7]=0x01;                      //参数3  第一个舵机ID号
	if(velocity_ID1>=0)
	{
		send_buff[8]=velocity_ID1;              //参数4  速度1的低8位
		send_buff[9]=velocity_ID1>>8;           //参数5  速度1的高8位
	}
	else                                       //不能用补码形式
	{
		send_buff[8]=-velocity_ID1;
		send_buff[9]=0x80;               //确认第一位方向
		send_buff[9]=send_buff[9]|(-velocity_ID1>>8);
	}
	send_buff[10]=0x02;                     //参数6  第二个舵机ID号
	if(velocity_ID2>=0)
	{
		send_buff[11]=velocity_ID2;             //参数7  速度2的低8位
		send_buff[12]=velocity_ID2>>8;          //参数8  速度2的高8位
	}
	else
	{
		send_buff[11]=-velocity_ID2;           
		send_buff[12]=0x80;
		send_buff[12]=send_buff[12]|(-velocity_ID2>>8);         
	}
	send_buff[13]=0x03;                     //参数9  第三个舵机ID号
	if(velocity_ID3>=0)
	{
		send_buff[14]=velocity_ID3;             //参数10  速度3的低8位
		send_buff[15]=velocity_ID3>>8;          //参数11  速度3的高8位
	}
	else
	{
		send_buff[14]=-velocity_ID3;       
		send_buff[15]=0x80;  
		send_buff[15]=send_buff[15]|(-velocity_ID3>>8);  
	}
	send_buff[16]=0x04;                     //参数9  第四个舵机ID号
	if(velocity_ID4>=0)
	{
		send_buff[17]=velocity_ID4;             //参数10  速度3的低8位
		send_buff[18]=velocity_ID4>>8;          //参数11  速度3的高8位
	}
	else
	{
		send_buff[17]=-velocity_ID4;       
		send_buff[18]=0x80;  
		send_buff[18]=send_buff[18]|(-velocity_ID4>>8);  
	}
	send_buff[19]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]+send_buff[16]+send_buff[17]+send_buff[18]);          //校验位
	USART3_SendBuff(send_buff,20);
}


void set_steering_engine_velocity_torque(int16_t torque_ID1,int16_t torque_ID2,int16_t torque_ID3,int16_t torque_ID4)             //设置三个舵机的速度
{
	uint8_t send_buff[20];                  //控制舵机速度发送帧
	send_buff[1]=send_buff[0]=0xFF;         //帧头
	send_buff[2]=0xFE;                      //ID地址 广播ID
	send_buff[3]=0x10;                      //数据长度  参数长度+2   16个参数
	send_buff[4]=0x83;                      //指令
	send_buff[5]=0x2C;                      //参数1  写数据的首地址
	send_buff[6]=0x02;                      //参数2  写取数据的长度  
	send_buff[7]=0x01;                      //参数3  第一个舵机ID号
	if(torque_ID1>=0)
	{
		send_buff[8]=torque_ID1;              //参数4  速度1的低8位
		send_buff[9]=torque_ID1>>8;           //参数5  速度1的高8位
	}
	else                                       //不能用补码形式
	{
		torque_ID1=-torque_ID1;
		torque_ID1+=1024;
		send_buff[8]=torque_ID1;              
		send_buff[9]=torque_ID1>>8;
	}
	send_buff[10]=0x02;                     //参数6  第二个舵机ID号
	if(torque_ID2>=0)
	{
		send_buff[11]=torque_ID2;             //参数7  速度2的低8位
		send_buff[12]=torque_ID2>>8;          //参数8  速度2的高8位
	}
	else
	{
		torque_ID2=-torque_ID2;
		torque_ID2+=1024;
		send_buff[11]=torque_ID2;              
		send_buff[12]=torque_ID2>>8;         
	}
	send_buff[13]=0x03;                     //参数9  第三个舵机ID号
	if(torque_ID3>=0)
	{
		send_buff[14]=torque_ID3;             //参数10  速度3的低8位
		send_buff[15]=torque_ID3>>8;          //参数11  速度3的高8位
	}
	else
	{
		torque_ID3=-torque_ID3;
		torque_ID3+=1024;
		send_buff[14]=torque_ID3;              
		send_buff[15]=torque_ID3>>8;  
	}
	send_buff[16]=0x04;                     //参数9  第四个舵机ID号
	if(torque_ID4>=0)
	{
		send_buff[17]=torque_ID4;             //参数10  速度3的低8位
		send_buff[18]=torque_ID4>>8;          //参数11  速度3的高8位
	}
	else
	{
		torque_ID4=-torque_ID4;
		torque_ID4+=1024;
		send_buff[17]=torque_ID4;              
		send_buff[18]=torque_ID4>>8; 
	}
	send_buff[19]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]+send_buff[16]+send_buff[17]+send_buff[18]);          //校验位
	USART3_SendBuff(send_buff,20);
}

//float pos_x=0;
//float pos_y=0; 
//float w_z=0;	
//float zangle=0; 
//float O_x=0,O_y=0,O_z=0;                //三轴绝对误差	
//u8 pos_count=0;
//u8 U2_Rceciv_error=1;                   //接收失败的次数
//void USART3_IRQHandler(void) {  	  
//		 static uint8_t ch; 
//	   static union{ 
//				uint8_t data[24];
//				float ActVal[6];
//     }posture; 
//		 static uint8_t count=0;
//     static uint8_t i=0;   
//		 OSIntEnter();
//		 if(USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET){
//         USART_ClearITPendingBit(USART3,USART_IT_RXNE); 
//				 ch=USART_ReceiveData(USART3);
//				 switch(count){
//              case 0:
//                  if(ch==0x0d)
//                      count++;
//                  else
//                      count=0;
//                  break;
//							case 1:
//                  if(ch==0x0a){
//										i=0;
//										count++;
//                  }
//                  else if(ch==0x0d); 
//									else
//                      count=0;
//                  break;
//						  case 2:
//                  posture.data[i]=ch;
//                  i++;
//                  if(i>=24){
//										i=0; 
//										count++;
//                  }
//                  break;
//							case 3:
//					  			if(ch==0x0a)
//										count++;
//                  else
//										count=0;
//                  break;   
//							case 4:
//									if(ch==0x0d){
//										if(++U2_Rceciv_error>5)
//										{
//											U2_Rceciv_error=0;
//											LED1=!LED1;
//									}
//										zangle=posture.ActVal[0];
//  									pos_x =posture.ActVal[3];
//										pos_y =posture.ActVal[4]; 
//										w_z   =posture.ActVal[5];
//									}
//									count=0;
//									break;
//							default:
//									count=0;
//							break;
//					}
//		 }									
//		 OSIntExit(); 
//}

//u8 Action_calibtation(void){
//	if(pos_count<250)
//	{
//		if(U2_Rceciv_error==0)
//		{
//			U2_Rceciv_error=1;
//			O_x+=pos_x;
//			O_y+=pos_y;
//			O_z+=w_z;
//			pos_count++;
//		}
//		return 1;
//	}
//	else
//	{
//		O_x*=0.004f;
//		O_y*=0.004f;
//		O_z*=0.004f;
//		return 0;
//	}
//}
