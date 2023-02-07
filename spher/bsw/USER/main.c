#include "delay.h"
#include "task.h"
#include "includes.h"
#include "can1.h"
#include "uart3.h"
#include "uart6.h"

int main(void){
	OS_ERR err;       //�������ϵͳ��������
	CPU_SR_ALLOC();   //��ջָ�붨��
	delay_init(168);  //ʱ�ӳ�ʼ��
	
	OSInit(&err);		//��ʼ��UCOSIII	
	OS_CRITICAL_ENTER();	//�����ٽ�����������������
	StartTaskCreate();   //������ʼ����
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

