//#include "lcd.h"
//#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "uart6.h"
#include "tim2.h"
#include "tim3.h"
#include "pstwo.h"
//#include "uart2.h"
//#include "uart6.h"
//#include "gpio.h"
//#include "24cxx.h"
#include "includes.h"

//�������ȼ�
#define START_TASK_PRIO		2
//�����ջ��С	
#define START_STK_SIZE 		128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];

#define INIT_TASK_PRIO		3
#define INIT_STK_SIZE 		128
OS_TCB InitTaskTCB;
CPU_STK INIT_TASK_STK[INIT_STK_SIZE];

#if EN_LCD_TASK
#define LCD_TASK_PRIO		4
#define LCD_STK_SIZE 		256
OS_TCB LCDTaskTCB;
CPU_STK LCD_TASK_STK[LCD_STK_SIZE];
#endif

#if EN_TELE_TASK
#define TELE_TASK_PRIO		5
#define TELE_STK_SIZE 		128
OS_TCB TELETaskTCB;
CPU_STK TELE_TASK_STK[TELE_STK_SIZE];
#endif

#if EN_CM_TASK
#define CM_TASK_PRIO	   5
#define CM_STK_SIZE 		1024
OS_TCB CMTaskTCB;
CPU_STK CM_TASK_STK[CM_STK_SIZE];
#endif

#if EN_Cylinder_TASK
#define Cylinder_TASK_PRIO		5
#define Cylinder_STK_SIZE 		128
OS_TCB CylinderTaskTCB;
CPU_STK Cylinder_TASK_STK[Cylinder_STK_SIZE];
#endif

#if EN_SphereMove_TASK
#define SphereMove_TASK_PRIO		4
#define SphereMove_STK_SIZE 		1024
OS_TCB SphereMoveTaskTCB;
CPU_STK SphereMove_TASK_STK[SphereMove_STK_SIZE];
#endif


//��ʼ������
void start_task(void *p_arg){
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	OS_CRITICAL_ENTER();	//�����ٽ���
						 
//������ʼ������
	OSTaskCreate(  (OS_TCB 	* )&InitTaskTCB,		
				         (CPU_CHAR	* )"Init task", 		
                 (OS_TASK_PTR )Init_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )INIT_TASK_PRIO,     	
                 (CPU_STK   * )&INIT_TASK_STK[0],	
                 (CPU_STK_SIZE)INIT_STK_SIZE/10,	
                 (CPU_STK_SIZE)INIT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);		

#if EN_SphereMove_TASK           //��������˶�����
	OSTaskCreate(  (OS_TCB 	* )&SphereMoveTaskTCB,		
				         (CPU_CHAR	* )"SphereMove task", 		
                 (OS_TASK_PTR )SphereMove_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )SphereMove_TASK_PRIO,     	
                 (CPU_STK   * )&SphereMove_TASK_STK[0],	
                 (CPU_STK_SIZE)SphereMove_STK_SIZE/10,	
                 (CPU_STK_SIZE)SphereMove_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif	
								 
#if EN_LCD_TASK
	OSTaskCreate(  (OS_TCB 	* )&LCDTaskTCB,		
				         (CPU_CHAR	* )"LCD task", 		
                 (OS_TASK_PTR )LCD_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LCD_TASK_PRIO,     	
                 (CPU_STK   * )&LCD_TASK_STK[0],	
                 (CPU_STK_SIZE)LCD_STK_SIZE/10,	
                 (CPU_STK_SIZE)LCD_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif			
								 
#if EN_TELE_TASK
	OSTaskCreate(  (OS_TCB 	* )&TELETaskTCB,		
				         (CPU_CHAR	* )"TeleCtrl task", 		
                 (OS_TASK_PTR )TeleCtrl_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TELE_TASK_PRIO,     	
                 (CPU_STK   * )&TELE_TASK_STK[0],	
                 (CPU_STK_SIZE)TELE_STK_SIZE/10,	
                 (CPU_STK_SIZE)TELE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif			
								 
#if EN_Cylinder_TASK
	//������������
	OSTaskCreate(  (OS_TCB 	* )&CylinderTaskTCB,		
				         (CPU_CHAR	* )"Cylinder task", 		
                 (OS_TASK_PTR )Cylinder_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )Cylinder_TASK_PRIO,     	
                 (CPU_STK   * )&Cylinder_TASK_STK[0],	
                 (CPU_STK_SIZE)Cylinder_STK_SIZE/10,	
                 (CPU_STK_SIZE)Cylinder_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);					 
#endif
#if EN_CM_TASK
	//����CM����
	OSTaskCreate(  (OS_TCB 	* )&CMTaskTCB,		
				         (CPU_CHAR	* )"CM task", 		
                 (OS_TASK_PTR )CM_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CM_TASK_PRIO,     	
                 (CPU_STK   * )&CM_TASK_STK[0],	
                 (CPU_STK_SIZE)CM_STK_SIZE/10,	
                 (CPU_STK_SIZE)CM_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);					 
#endif
//	OS_TaskSuspend((OS_TCB*)&LCDTaskTCB,&err);
//	OS_TaskSuspend((OS_TCB*)&TELETaskTCB,&err);         //����ң������
//	OS_TaskSuspend((OS_TCB*)&CMTaskTCB,&err);          //�����������
//	OS_TaskSuspend((OS_TCB*)&CylinderTaskTCB,&err);   //������������
	OS_TaskSuspend((OS_TCB*)&SphereMoveTaskTCB,&err);   //��������˶�����
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�����ٽ���   
}

void StartTaskCreate(void){
	OS_ERR err;
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//������ƿ�
								 (CPU_CHAR * )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK  * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY)0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
}

void Init_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
	delay_init(168);
	TIM2_Init();
	//TIM3_Init();
//	USART1_Init();
//	USART2_Init();
	USART3_Init();
	USART6_Config();	                    //���������ݶ�ȡ
//	PS2_Init();									    //=====ps2�����˿ڳ�ʼ��
//	PS2_SetInit();		 					    //=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
//		TEL_USART6_Config();  //ң��������
//		CANx_Init();          //CAN��������
//		USART1_Init();				//CCD
//		USART2_Init();        //ǰ��
//		USART3_Init();				//���ض�λģ��(UART3)����
//		GPIO_Config();
//		LCD_Init();           //LCD����
//		AT24CXX_Init();

//		OS_TaskResume((OS_TCB*)&LCDTaskTCB,&err);
	delay_ms(500);
	OS_TaskResume((OS_TCB*)&SphereMoveTaskTCB,&err);   //��������˶�����
	OS_TaskSuspend((OS_TCB*)&InitTaskTCB,&err);		   //��������		
}



//u8 Tele_Receiv_error=11;
//void TeleCtrl_task(void *p_arg){
//	OS_ERR err;
//	p_arg = p_arg;
//	while(1){
//		if(DMA2->LISR&DMA_IT_TCIF2){    //DMA�������
//				 DMA2->LIFCR=DMA_FLAG_TCIF2&(uint32_t)0x0F7D0F7D;  //���������ɱ�־λ
//				 Tele_Receiv_error=0;
//				 TelCtrlData.right_x = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;//��x  1684~1024`364
//				 TelCtrlData.right_y = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;//��y
//				 TelCtrlData.left_x = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff; //��x
//				 TelCtrlData.left_y = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;      //��y
//				 TelCtrlData.switch_l = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;  
//				 TelCtrlData.switch_r = ((sbus_rx_buffer[5] >> 4)& 0x0003);       
//				 TelCtrlData.mouse_x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);
//				 TelCtrlData.mouse_y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);
//				 TelCtrlData.mouse_z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);		
//				 TelCtrlData.mouse_l = sbus_rx_buffer[12];   //���
//				 TelCtrlData.mouse_r = sbus_rx_buffer[13];  //�Ҽ�
//				 TelCtrlData.key = sbus_rx_buffer[14]| (sbus_rx_buffer[15] << 8);//���̰���	 
//		}
//		OSTimeDly(7,OS_OPT_TIME_PERIODIC,&err); //��ʱ7ms	
//	}
//}
