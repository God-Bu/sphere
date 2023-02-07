#include "uart3.h"
#include "sys.h"
//#include "gpio.h"
//////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
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
    usart.USART_WordLength = USART_WordLength_8b;//�ֳ�8B
    usart.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    usart.USART_Parity = USART_Parity_No;//��żУ��λ
    usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;//����ģʽ  ����ģʽ
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart);
	
	DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);//�������ݼĴ�����ַ
    dma.DMA_Memory0BaseAddr = (uint32_t)rx_buffer;//�ڴ��ַ
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���ݴ��䷽��  ���赽�ڴ�
    dma.DMA_BufferSize = 22;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ�����ַ����
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;//ѭ���ɼ�
    dma.DMA_Priority = DMA_Priority_VeryHigh;//DMAͨ�����ȼ�
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//�����ȳ���FIFO��
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//ѡ��FIFO��ֵ
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ����������
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ����������
    DMA_Init(DMA1_Stream1,&dma);
    DMA_Cmd(DMA1_Stream1,ENABLE);
	
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//��������DMA���չ���
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	USART_Cmd(USART3,ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);
	
}

void USART3_SendBuff(uint8_t *addr,uint8_t count_n)   //��һ����ַ��ʼ��count_n������
{
	for(int i=0;i<count_n;i++)
	{
		USART_ClearFlag(USART3,USART_FLAG_TC);
		USART_SendData(USART3,*(addr++));
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//�ж��Ƿ������
	}
}

void get_steering_engine_status(void)   //��ȡID1-ID4�Ķ��״̬
{
	uint8_t send_buff[12];                 //��ѯ����ڴ淢��֡
	send_buff[1]=send_buff[0]=0xFF;        //֡ͷ
	send_buff[2]=0xFE;                     //ID��ַ �㲥ID
	send_buff[3]=0x08;                     //���ݳ���  ��������+2
	send_buff[4]=0x82;                     //ָ��
	send_buff[5]=0x38;                     //����1  ��ȡ���ݵ��׵�ַ
	send_buff[6]=0x0F;                     //����2  ��ȡ���ݵĳ���  
	send_buff[7]=0x01;                     //����3  ��һ�����ID��
	send_buff[8]=0x02;                     //����4  �ڶ������ID��
	send_buff[9]=0x03;                     //����5  ���������ID��
	send_buff[10]=0x04;                     //����6  ���������ID��
	send_buff[11]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+
					send_buff[6]+send_buff[7]+send_buff[8]+send_buff[9]+send_buff[10]);                     //У��λ
	USART3_SendBuff(send_buff,12);
}


void set_steering_engine_velocity(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3)             //��������������ٶ�
{
	uint8_t send_buff[17];                  //���ƶ���ٶȷ���֡
	send_buff[1]=send_buff[0]=0xFF;         //֡ͷ
	send_buff[2]=0xFE;                      //ID��ַ �㲥ID
	send_buff[3]=0x0D;                      //���ݳ���  ��������+2   11������
	send_buff[4]=0x83;                      //ָ��
	send_buff[5]=0x2E;                      //����1  д���ݵ��׵�ַ
	send_buff[6]=0x02;                      //����2  дȡ���ݵĳ���  
	send_buff[7]=0x01;                      //����3  ��һ�����ID��
	if(velocity_ID1>=0)
	{
		send_buff[8]=velocity_ID1;              //����4  �ٶ�1�ĵ�8λ
		send_buff[9]=velocity_ID1>>8;           //����5  �ٶ�1�ĸ�8λ
	}
	else                                       //�����ò�����ʽ
	{
		send_buff[8]=-velocity_ID1;
		send_buff[9]=0x80;               //ȷ�ϵ�һλ����
		send_buff[9]=send_buff[9]|(-velocity_ID1>>8);
	}
	send_buff[10]=0x02;                     //����6  �ڶ������ID��
	if(velocity_ID2>=0)
	{
		send_buff[11]=velocity_ID2;             //����7  �ٶ�2�ĵ�8λ
		send_buff[12]=velocity_ID2>>8;          //����8  �ٶ�2�ĸ�8λ
	}
	else
	{
		send_buff[11]=-velocity_ID2;           
		send_buff[12]=0x80;
		send_buff[12]=send_buff[12]|(-velocity_ID2>>8);         
	}
	send_buff[13]=0x03;                     //����9  ���������ID��
	if(velocity_ID3>=0)
	{
		send_buff[14]=velocity_ID3;             //����10  �ٶ�3�ĵ�8λ
		send_buff[15]=velocity_ID3>>8;          //����11  �ٶ�3�ĸ�8λ
	}
	else
	{
		send_buff[14]=-velocity_ID3;       
		send_buff[15]=0x80;  
		send_buff[15]=send_buff[15]|(-velocity_ID3>>8);  
	}
	send_buff[16]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]);          //У��λ
	USART3_SendBuff(send_buff,17);
}

void set_steering_engine_acceleration(uint8_t acceleration_ID1,uint8_t acceleration_ID2,uint8_t acceleration_ID3)             //������������ļ��ٶ�
{
	uint8_t send_buff[14];                  //���ƶ���ٶȷ���֡
	send_buff[1]=send_buff[0]=0xFF;         //֡ͷ
	send_buff[2]=0xFE;                      //ID��ַ �㲥ID
	send_buff[3]=0x0A;                      //���ݳ���  ��������+2   11������
	send_buff[4]=0x83;                      //ָ��
	send_buff[5]=0x29;                      //����1  д���ݵ��׵�ַ
	send_buff[6]=0x01;                      //����2  дȡ���ݵĳ���  
	send_buff[7]=0x01;                      //����3  ��һ�����ID��
	send_buff[8]=acceleration_ID1;          //����4  ���ٶ�1
	send_buff[9]=0x02;                      //����5  �ڶ������ID��
	send_buff[10]=acceleration_ID2;         //����6  ���ٶ�2
	send_buff[11]=0x03;                     //����7  ���������ID��
	send_buff[12]=acceleration_ID3;         //����8  ���ٶ�3
	send_buff[13]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+
					send_buff[8]+send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]);          //У��λ
	USART3_SendBuff(send_buff,14);
}



void USART3_IRQHandler(void) {  	  
	 u8 Current_NDTR=0; 
	 OSIntEnter();
	
	 if (USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)  
   {  
		 //����жϱ�־    
		 (void)USART3->SR;   
		 (void)USART3->DR;
		 
		 if(DMA_GetFlagStatus(DMA1_Stream1,DMA_IT_TCIF1) == SET)        //DMA�������
		 {
			 DMA_ClearFlag(DMA1_Stream1, DMA_IT_TCIF1);         //���������ɱ�־λ			 
			 DMA_Cmd(DMA1_Stream1, DISABLE);                      //�ر�DMA  
			 Current_NDTR=DMA1_Stream1->NDTR;                     //��ȡʣ�໺����
			 DMA1_Stream1->NDTR=22;                               //����DMA����ֵ  
			 DMA_Cmd(DMA1_Stream1, ENABLE);                       //����DMA	
		 
			 if(Current_NDTR==0x01)       //��ʣ��1�������ֽڣ�˵����һ֡��������
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
	if(rx_buffer[0]!=0xFF || rx_buffer[1]!=0xFF)return;           //֡ͷ���� 
	for(int i=2;i<20;i++)
	{
		check_sum+=rx_buffer[i];
	}
	check_sum=~check_sum;
	if( check_sum==rx_buffer[20] )             //������ȷ
	{
		switch(rx_buffer[2])                      //�ж����ĸ�ID�����������Ϣ
		{
			case 0x01:
			{
				Steer_engine_ID1.position = (rx_buffer[6]<<8) | rx_buffer[5];         //λ��  4096(��)�ֱ���
				if((rx_buffer[8]>>7)==0)                    //����
				{
					Steer_engine_ID1.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //�ٶ�  ��λ50��/s
				}
				else
				{
					Steer_engine_ID1.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID1.load = (rx_buffer[10]<<8) | rx_buffer[9];            //����  ���������ѹռ�ձ� ��λ0.1%
				Steer_engine_ID1.Voltage = rx_buffer[11];                             //��ѹ  ��λ0.1V
				Steer_engine_ID1.T = rx_buffer[12];                                   //�¶�  ��λ1���϶�
				Steer_engine_ID1.flag_Awrite = rx_buffer[13];                         //�첽д��ʶ
				Steer_engine_ID1.flag_error = rx_buffer[14];                          //��������ʶ  0��ʾ�޴���
				Steer_engine_ID1.flag_move = rx_buffer[15];                           //����ƶ���ʶ  1��ʾ�˶� 0��ʾֹͣ
				Steer_engine_ID1.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //Ŀ�ꣿ
				Steer_engine_ID1.current = (rx_buffer[19]<<8) | rx_buffer[18];        //��ǰ���� ��λ6.5mA
			}break;
			case 0x02:
			{
				Steer_engine_ID2.position = (rx_buffer[6]<<8) | rx_buffer[5];         //λ��  4096(��)�ֱ���
				if((rx_buffer[8]>>7)==0)                    //����
				{
					Steer_engine_ID2.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //�ٶ�  ��λ50��/s
				}
				else
				{
					Steer_engine_ID2.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}    
				Steer_engine_ID2.load = (rx_buffer[10]<<8) | rx_buffer[9];            //����  ���������ѹռ�ձ� ��λ0.1%
				Steer_engine_ID2.Voltage = rx_buffer[11];                             //��ѹ  ��λ0.1V
				Steer_engine_ID2.T = rx_buffer[12];                                   //�¶�  ��λ1���϶�
				Steer_engine_ID2.flag_Awrite = rx_buffer[13];                         //�첽д��ʶ
				Steer_engine_ID2.flag_error = rx_buffer[14];                          //��������ʶ  0��ʾ�޴���
				Steer_engine_ID2.flag_move = rx_buffer[15];                           //����ƶ���ʶ  1��ʾ�˶� 0��ʾֹͣ
				Steer_engine_ID2.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //Ŀ�ꣿ
				Steer_engine_ID2.current = (rx_buffer[19]<<8) | rx_buffer[18];        //��ǰ���� ��λ6.5mA
			}break;
			case 0x03:
			{
				Steer_engine_ID3.position = (rx_buffer[6]<<8) | rx_buffer[5];         //λ��  4096(��)�ֱ���
				if((rx_buffer[8]>>7)==0)                    //����
				{
					Steer_engine_ID3.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //�ٶ�  ��λ50��/s
				}
				else
				{
					Steer_engine_ID3.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID3.load = (rx_buffer[10]<<8) | rx_buffer[9];            //����  ���������ѹռ�ձ� ��λ0.1%
				Steer_engine_ID3.Voltage = rx_buffer[11];                             //��ѹ  ��λ0.1V
				Steer_engine_ID3.T = rx_buffer[12];                                   //�¶�  ��λ1���϶�
				Steer_engine_ID3.flag_Awrite = rx_buffer[13];                         //�첽д��ʶ
				Steer_engine_ID3.flag_error = rx_buffer[14];                          //��������ʶ  0��ʾ�޴���
				Steer_engine_ID3.flag_move = rx_buffer[15];                           //����ƶ���ʶ  1��ʾ�˶� 0��ʾֹͣ
				Steer_engine_ID3.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //Ŀ�ꣿ
				Steer_engine_ID3.current = (rx_buffer[19]<<8) | rx_buffer[18];        //��ǰ���� ��λ6.5mA
			}break;
			case 0x04:
			{
				Steer_engine_ID4.position = (rx_buffer[6]<<8) | rx_buffer[5];         //λ��  4096(��)�ֱ���
				if((rx_buffer[8]>>7)==0)                    //����
				{
					Steer_engine_ID4.velocity = (rx_buffer[8]<<8) | rx_buffer[7];         //�ٶ�  ��λ50��/s
				}
				else
				{
					Steer_engine_ID4.velocity=-(((rx_buffer[8] & 0x7F)<<8) | rx_buffer[7]);
				}
				Steer_engine_ID4.load = (rx_buffer[10]<<8) | rx_buffer[9];            //����  ���������ѹռ�ձ� ��λ0.1%
				Steer_engine_ID4.Voltage = rx_buffer[11];                             //��ѹ  ��λ0.1V
				Steer_engine_ID4.T = rx_buffer[12];                                   //�¶�  ��λ1���϶�
				Steer_engine_ID4.flag_Awrite = rx_buffer[13];                         //�첽д��ʶ
				Steer_engine_ID4.flag_error = rx_buffer[14];                          //��������ʶ  0��ʾ�޴���
				Steer_engine_ID4.flag_move = rx_buffer[15];                           //����ƶ���ʶ  1��ʾ�˶� 0��ʾֹͣ
				Steer_engine_ID4.velocity = (rx_buffer[17]<<8) | rx_buffer[16];       //Ŀ�ꣿ
				Steer_engine_ID4.current = (rx_buffer[19]<<8) | rx_buffer[18];        //��ǰ���� ��λ6.5mA
			}break;
			default: break;
		}
	}
	check_sum=0;               //У��ֵ����
}

void set_steering_engine_velocity_four(int16_t velocity_ID1,int16_t velocity_ID2,int16_t velocity_ID3,int16_t velocity_ID4)             //��������������ٶ�
{
	uint8_t send_buff[20];                  //���ƶ���ٶȷ���֡
	send_buff[1]=send_buff[0]=0xFF;         //֡ͷ
	send_buff[2]=0xFE;                      //ID��ַ �㲥ID
	send_buff[3]=0x10;                      //���ݳ���  ��������+2   16������
	send_buff[4]=0x83;                      //ָ��
	send_buff[5]=0x2E;                      //����1  д���ݵ��׵�ַ
	send_buff[6]=0x02;                      //����2  дȡ���ݵĳ���  
	send_buff[7]=0x01;                      //����3  ��һ�����ID��
	if(velocity_ID1>=0)
	{
		send_buff[8]=velocity_ID1;              //����4  �ٶ�1�ĵ�8λ
		send_buff[9]=velocity_ID1>>8;           //����5  �ٶ�1�ĸ�8λ
	}
	else                                       //�����ò�����ʽ
	{
		send_buff[8]=-velocity_ID1;
		send_buff[9]=0x80;               //ȷ�ϵ�һλ����
		send_buff[9]=send_buff[9]|(-velocity_ID1>>8);
	}
	send_buff[10]=0x02;                     //����6  �ڶ������ID��
	if(velocity_ID2>=0)
	{
		send_buff[11]=velocity_ID2;             //����7  �ٶ�2�ĵ�8λ
		send_buff[12]=velocity_ID2>>8;          //����8  �ٶ�2�ĸ�8λ
	}
	else
	{
		send_buff[11]=-velocity_ID2;           
		send_buff[12]=0x80;
		send_buff[12]=send_buff[12]|(-velocity_ID2>>8);         
	}
	send_buff[13]=0x03;                     //����9  ���������ID��
	if(velocity_ID3>=0)
	{
		send_buff[14]=velocity_ID3;             //����10  �ٶ�3�ĵ�8λ
		send_buff[15]=velocity_ID3>>8;          //����11  �ٶ�3�ĸ�8λ
	}
	else
	{
		send_buff[14]=-velocity_ID3;       
		send_buff[15]=0x80;  
		send_buff[15]=send_buff[15]|(-velocity_ID3>>8);  
	}
	send_buff[16]=0x04;                     //����9  ���ĸ����ID��
	if(velocity_ID4>=0)
	{
		send_buff[17]=velocity_ID4;             //����10  �ٶ�3�ĵ�8λ
		send_buff[18]=velocity_ID4>>8;          //����11  �ٶ�3�ĸ�8λ
	}
	else
	{
		send_buff[17]=-velocity_ID4;       
		send_buff[18]=0x80;  
		send_buff[18]=send_buff[18]|(-velocity_ID4>>8);  
	}
	send_buff[19]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]+send_buff[16]+send_buff[17]+send_buff[18]);          //У��λ
	USART3_SendBuff(send_buff,20);
}


void set_steering_engine_velocity_torque(int16_t torque_ID1,int16_t torque_ID2,int16_t torque_ID3,int16_t torque_ID4)             //��������������ٶ�
{
	uint8_t send_buff[20];                  //���ƶ���ٶȷ���֡
	send_buff[1]=send_buff[0]=0xFF;         //֡ͷ
	send_buff[2]=0xFE;                      //ID��ַ �㲥ID
	send_buff[3]=0x10;                      //���ݳ���  ��������+2   16������
	send_buff[4]=0x83;                      //ָ��
	send_buff[5]=0x2C;                      //����1  д���ݵ��׵�ַ
	send_buff[6]=0x02;                      //����2  дȡ���ݵĳ���  
	send_buff[7]=0x01;                      //����3  ��һ�����ID��
	if(torque_ID1>=0)
	{
		send_buff[8]=torque_ID1;              //����4  �ٶ�1�ĵ�8λ
		send_buff[9]=torque_ID1>>8;           //����5  �ٶ�1�ĸ�8λ
	}
	else                                       //�����ò�����ʽ
	{
		torque_ID1=-torque_ID1;
		torque_ID1+=1024;
		send_buff[8]=torque_ID1;              
		send_buff[9]=torque_ID1>>8;
	}
	send_buff[10]=0x02;                     //����6  �ڶ������ID��
	if(torque_ID2>=0)
	{
		send_buff[11]=torque_ID2;             //����7  �ٶ�2�ĵ�8λ
		send_buff[12]=torque_ID2>>8;          //����8  �ٶ�2�ĸ�8λ
	}
	else
	{
		torque_ID2=-torque_ID2;
		torque_ID2+=1024;
		send_buff[11]=torque_ID2;              
		send_buff[12]=torque_ID2>>8;         
	}
	send_buff[13]=0x03;                     //����9  ���������ID��
	if(torque_ID3>=0)
	{
		send_buff[14]=torque_ID3;             //����10  �ٶ�3�ĵ�8λ
		send_buff[15]=torque_ID3>>8;          //����11  �ٶ�3�ĸ�8λ
	}
	else
	{
		torque_ID3=-torque_ID3;
		torque_ID3+=1024;
		send_buff[14]=torque_ID3;              
		send_buff[15]=torque_ID3>>8;  
	}
	send_buff[16]=0x04;                     //����9  ���ĸ����ID��
	if(torque_ID4>=0)
	{
		send_buff[17]=torque_ID4;             //����10  �ٶ�3�ĵ�8λ
		send_buff[18]=torque_ID4>>8;          //����11  �ٶ�3�ĸ�8λ
	}
	else
	{
		torque_ID4=-torque_ID4;
		torque_ID4+=1024;
		send_buff[17]=torque_ID4;              
		send_buff[18]=torque_ID4>>8; 
	}
	send_buff[19]=~(send_buff[2]+send_buff[3]+send_buff[4]+send_buff[5]+send_buff[6]+send_buff[7]+send_buff[8]+
	                send_buff[9]+send_buff[10]+send_buff[11]+send_buff[12]+send_buff[13]+send_buff[14]+send_buff[15]+send_buff[16]+send_buff[17]+send_buff[18]);          //У��λ
	USART3_SendBuff(send_buff,20);
}

//float pos_x=0;
//float pos_y=0; 
//float w_z=0;	
//float zangle=0; 
//float O_x=0,O_y=0,O_z=0;                //����������	
//u8 pos_count=0;
//u8 U2_Rceciv_error=1;                   //����ʧ�ܵĴ���
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
