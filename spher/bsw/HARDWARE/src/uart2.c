#include "uart2.h"
#include "uart3.h"
#include "sys.h"
#include "gpio.h"
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

char UWB_rx_buffer[100];

u8 ch;
u8 count=0;

UWB_DATA UWB_data;

void USART2_Init(void){                 //��ǰ��ͨ��
	USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef   dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 

	gpio.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6; 
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;	
	gpio.GPIO_OType = GPIO_OType_PP; 
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&gpio); 

	nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    USART_DeInit(USART2);
    usart.USART_BaudRate = 115200;                //UART2 115200bps
    usart.USART_WordLength = USART_WordLength_8b;//�ֳ�8B
    usart.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    usart.USART_Parity = USART_Parity_No;//��żУ��λ
    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//����ģʽ  ����ģʽ
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2,&usart);
	
//	DMA_DeInit(DMA1_Stream5);
//    dma.DMA_Channel= DMA_Channel_4;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//�������ݼĴ�����ַ
//    dma.DMA_Memory0BaseAddr = (uint32_t)UWB_rx_buffer;//�ڴ��ַ
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���ݴ��䷽��  ���赽�ڴ�
//    dma.DMA_BufferSize = 87;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ�����ַ����
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;//ѭ���ɼ�
//    dma.DMA_Priority = DMA_Priority_VeryHigh;//DMAͨ�����ȼ�
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//�����ȳ���FIFO��
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//ѡ��FIFO��ֵ
//    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ����������
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ����������
//    DMA_Init(DMA1_Stream5,&dma);
//    DMA_Cmd(DMA1_Stream5,ENABLE);
	
//	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//��������DMA���չ���
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڿ����ж�
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	USART_Cmd(USART2,ENABLE);
//	DMA_Cmd(DMA1_Stream5,ENABLE);
	
}


void USART2_IRQHandler(void) {
//	u8 ch;
    OSIntEnter();
	u8 Current_NDTR=0;
	u8 tem;
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
	{
		tem=USART_ReceiveData(USART2);
		UWB_rx_buffer[count++]=tem;
	}
	
	if(USART_GetITStatus(USART2,USART_IT_IDLE) != RESET)
	{
		//����жϱ�־    
		 (void)USART2->SR;   
		 (void)USART2->DR;
		ch=count;
		count=0;
		
		//USART3_SendBuff(UWB_rx_buffer,sizeof(UWB_rx_buffer));
		
//		char data[]="24.254,2.90,1.23,3.44,1.10,-0.023926,0.189014,9.874170,0.007980,0.001596,0.016492,T0\r\n";
//		sscanf(data, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%s\r\n", 
//	&UWB_data.time, &UWB_data.distance_A0, &UWB_data.distance_A1, &UWB_data.distance_A2, &UWB_data.distance_A3, 
//	&UWB_data.Acc_x, &UWB_data.Acc_y, &UWB_data.Acc_z, &UWB_data.Gyrc_x, &UWB_data.Gyrc_y, &UWB_data.Gyrc_z,&UWB_data.tag_number);
		//UWB_Data_Process(UWB_rx_buffer);
		
		
		 
//		 if(DMA_GetFlagStatus(DMA1_Stream5,DMA_IT_TCIF5) == SET)        //DMA�������
//		 {
//			 
//			 USART3_SendBuff(UWB_rx_buffer,sizeof(UWB_rx_buffer));
//			 
//			 DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF5);         //���������ɱ�־λ			 
//			 DMA_Cmd(DMA1_Stream5, DISABLE);                      //�ر�DMA  
//			 Current_NDTR=DMA1_Stream5->NDTR;                     //��ȡʣ�໺����
//			 DMA1_Stream5->NDTR=87;                               //����DMA����ֵ  
//			 DMA_Cmd(DMA1_Stream5, ENABLE);                       //����DMA	
//			 
//			 
//			 
//			 //UWB_Data_Process(UWB_rx_buffer);              //����Ϊ������֡  �ֽ��������ж� ֱ�ӿ�ʼ����
//			 
//			 ch=Current_NDTR;
//			 
//			 
////		 
////			 if(Current_NDTR==0)       //��ʣ��1�������ֽڣ�˵����һ֡��������
////			 {
////				 UWB_Data_Process(UWB_rx_buffer);
////			 }	 
//			 
//		 }		
	}
	
//	if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET)
//	{
//		USART_ClearITPendingBit(USART2,USART_IT_RXNE); 
//		ch=USART_ReceiveData(USART2);
//	}									
	OSIntExit(); 
}

void UWB_Data_Process(char *rx_buffer)
{
	sscanf(rx_buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%s\r\n", 
	&UWB_data.time, &UWB_data.distance_A0, &UWB_data.distance_A1, &UWB_data.distance_A2, &UWB_data.distance_A3, 
	&UWB_data.Acc_x, &UWB_data.Acc_y, &UWB_data.Acc_z, &UWB_data.Gyrc_x, &UWB_data.Gyrc_y, &UWB_data.Gyrc_z,&UWB_data.tag_number);
}


//u8 fmessage=0x00;
//float error_x=0.0f;
//u8 pick_recflag=0;

//void USART2_IRQHandler(void) {  	
//		 static u8 count=0;
//		 static union{ 
//				u8 cdata[8];
//				float fdata[2];
//		 }f_data;
//	   OSIntEnter();
//		 if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET)
//		 {
//         USART_ClearITPendingBit(USART2,USART_IT_RXNE); 
//				 if(count==0)
//				 {
//					 if(USART_ReceiveData(USART2)==0x0a)
//						 count++;
//					 else
//						 count=0;
//				 }
//				 else
//				 {
//					 f_data.cdata[count]=USART_ReceiveData(USART2);
//					 if(count<7)
//							count++;
//					 else
//					 {
//						 fmessage = f_data.cdata[1];
//						 if((fmessage&0x20)==0x20)
//						 {
//							 pick_recflag=1; 
//							 error_x=f_data.fdata[1];
//						 }
//						 count=0;
//					 }
//				 }
//		 }									
//		 OSIntExit(); 
//}
