#include "dynamics.h"
#include "math.h"
#include "uart6.h"
#include "uart3.h"


#define centroid_raw_x 0                     //���ĵĳ�ʼxyzλ��
#define centroid_raw_y 0
#define centroid_raw_z -85                   //mm

#define pi 3.1415926                  //�������� Բ����
#define R 0.15                         //����⾶  m
#define r 0.041                        //ȫ���ְ뾶 m
#define d 0.003                           //��Ǻ��    m
#define RD (R-d)                      //����ھ�   m

#define m1 0.95                       //������� kg                      ����ѧ��������
#define m2 0.95                       //С������ kg
#define J1 1.4060                      //��ǹ��� kg*m*m
#define J2x 0.4783                      //С��xx���� kg*m*m
#define J2y 0.4782                      //С��yy���� kg*m*m
#define J2z 0.7183                      //С��zz���� kg*m*m
#define g 9.8                         //�������ٶ�




float centroid_reall_x(float Angle_x,float Angle_y,float Angle_z)          //����Ϊm
{
	float position_x;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	//����x��y��Ϊ0����д��ʡ����ʱ��
	//position_x=centroid_raw_z*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - centroid_raw_y*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)) + centroid_raw_x*cos(t2)*cos(t3);
	position_x=centroid_raw_z*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2));
	return position_x*0.001;
}


float centroid_reall_y(float Angle_x,float Angle_y,float Angle_z)          //����Ϊm
{
	float position_y;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	//����x��y��Ϊ0����д��ʡ����ʱ��
	//position_y=centroid_raw_y*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)) - centroid_raw_z*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + centroid_raw_x*cos(t2)*sin(t3);
	position_y=- centroid_raw_z*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3));
	return position_y*0.001;
}

float centroid_reall_z(float Angle_x,float Angle_y,float Angle_z)          //����Ϊm
{
	float position_z;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	//����x��y��Ϊ0����д��ʡ����ʱ��
	//position_z=centroid_raw_z*cos(t1)*cos(t2) - centroid_raw_x*sin(t2) + centroid_raw_y*cos(t2)*sin(t1);
	position_z=centroid_raw_z*cos(t1)*cos(t2);
	return position_z*0.001;
}

float U3_V()                                                   //�����ٶ�u3 
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	result= - t22*sin(t3) + t11*cos(t2)*cos(t3);
	
	return result;
}

float U4_V()                                                   //�����ٶ�u4
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	result= t22*cos(t3) + t11*cos(t2)*sin(t3);
	
	return result;
}

float U5_V()                                                   //�����ٶ�u4
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	result= t33 - t11*sin(t2);
	
	return result;
}



float U3_ACC()                                                   //�����ٶ�u3�ĵ���
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	float t111=Acc_angle_Data.Acc_angle_x*pi/180;
	float t222=Acc_angle_Data.Acc_angle_y*pi/180;
	float t333=Acc_angle_Data.Acc_angle_z*pi/180;
	result= - t222*sin(t3) - t22*t33*cos(t3) + t111*cos(t2)*cos(t3) + t11*(-t22*sin(t2)*cos(t3) - t33*cos(t2)*sin(t3));
	
	return result;
}

float U4_ACC()                                                   //�����ٶ�u4�ĵ���
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	float t111=Acc_angle_Data.Acc_angle_x*pi/180;
	float t222=Acc_angle_Data.Acc_angle_y*pi/180;
	float t333=Acc_angle_Data.Acc_angle_z*pi/180;
	result= t222*cos(t3) - t22*t33*sin(t3) + t111*cos(t2)*sin(t3) + t11*(-t22*sin(t2)*sin(t3) + t33*cos(t2)*cos(t3));
	
	return result;
}

float U5_ACC()                                                   //�����ٶ�u5�ĵ���
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float t11=Gyrc_Data.Gyrc_x*pi/180;
	float t22=Gyrc_Data.Gyrc_y*pi/180;
	float t33=Gyrc_Data.Gyrc_z*pi/180;
	float t111=Acc_angle_Data.Acc_angle_x*pi/180;
	float t222=Acc_angle_Data.Acc_angle_y*pi/180;
	float t333=Acc_angle_Data.Acc_angle_z*pi/180;
	result= t333 - t111*sin(t2) - t11*t22*cos(t2);
	
	return result;
}



float B_row1()                                                   //B����ĵ�һ��
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float u3=U3_V();
	float u4=U4_V();
	float u5=U5_V();
	float u33=U3_ACC();
	float u44=U4_ACC();
	float u55=U5_ACC();
	
	result=-m2*g*reall_y + m2*(-reall_x*u44 + reall_y*u33 + reall_y*u5*u4 - reall_z*u4*u4 + reall_x*u5*u3 - reall_z*u3*u3)*reall_y + J2x*u33 + (J2z-J2y)*u5*u4;
	
	return result;
}

float B_row2()                                                   //B����ĵڶ���
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float u3=U3_V();
	float u4=U4_V();
	float u5=U5_V();
	float u33=U3_ACC();
	float u44=U4_ACC();
	float u55=U5_ACC();
	
	result=m2*g*reall_x - m2*(-reall_x*u44 + reall_y*u33 + reall_y*u5*u4 - reall_z*u4*u4 + reall_x*u5*u3 - reall_z*u3*u3)*reall_x + J2y*u44 + (J2x-J2z)*u5*u3;
	
	return result;
}


float B_row3()                                                   //B����ĵ�3��
{
	float result;
	float u3=U3_V();
	float u4=U4_V();
	float u5=U5_V();
	
	result=(J2y-J2x)*u5*u4;
	
	return result;
}

float A11()                                                   //A�����a11Ԫ��
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	
	result=-m2*reall_z*R - reall_z*(J1 + m1*R*R + m2*R*R)/R;
	
	return result;
}

float A22()                                                   //A�����a22Ԫ��
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	
	result=-m2*reall_z*R - reall_z*(J1 + m1*R*R + m2*R*R)/R;
	
	return result;
}

float A31()                                                   //A�����a31Ԫ��
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	
	result=m2*reall_x*R - reall_x*(J1 + m1*R*R + m2*R*R)/R;
	
	return result;
}

float A32()                                                   //A�����a32Ԫ��
{
	float result;
	float reall_x=centroid_reall_x(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_y=centroid_reall_y(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	float reall_z=centroid_reall_z(Angle_Data.Angle_x, Angle_Data.Angle_y, Angle_Data.Angle_z);
	
	result=m2*reall_y*R + reall_y*(J1 + m1*R*R + m2*R*R)/R;
	
	return result;
}

float A33()                                                   //A�����a33Ԫ��
{
	float result;
	result=J2z;
	
	return result;
}

float reall_U1()                                           //ʵ�ʵĹ����ٶ�u1  ͨ���˶�ѧ��ʽ�������
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float q1=Steer_engine_ID1.velocity*pi/2048;
	float q2=Steer_engine_ID1.velocity*pi/2048;
	float q3=Steer_engine_ID1.velocity*pi/2048;
	float q4=Steer_engine_ID1.velocity*pi/2048;
	result=(r*(q1*sin(t3) + q2*sin(t3) - q3*sin(t3) - q4*sin(t3) - q1*cos(t2)*cos(t3) + q2*cos(t2)*cos(t3) + q3*cos(t2)*cos(t3) - q4*cos(t2)*cos(t3)))/(2*RD);
	
	return result;	
}

float reall_U2()                                           //ʵ�ʵĹ����ٶ�u2  ͨ���˶�ѧ��ʽ�������
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float q1=Steer_engine_ID1.velocity*pi/2048;
	float q2=Steer_engine_ID1.velocity*pi/2048;
	float q3=Steer_engine_ID1.velocity*pi/2048;
	float q4=Steer_engine_ID1.velocity*pi/2048;
	result=-(r*(q1*cos(t3) + q2*cos(t3) - q3*cos(t3) - q4*cos(t3) + q1*cos(t2)*sin(t3) - q2*cos(t2)*sin(t3) - q3*cos(t2)*sin(t3) + q4*cos(t2)*sin(t3)))/(2*RD);
	
	return result;	
}

float reall_U5()                                           //ʵ�ʵĹ����ٶ�u5  ͨ���˶�ѧ��ʽ�������
{
	float result;
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	float q1=Steer_engine_ID1.velocity*pi/2048;
	float q2=Steer_engine_ID1.velocity*pi/2048;
	float q3=Steer_engine_ID1.velocity*pi/2048;
	float q4=Steer_engine_ID1.velocity*pi/2048;
	result=(r*(sqrt(2)*q1 + sqrt(2)*q2 + sqrt(2)*q3 + sqrt(2)*q4 + 2*q1*sin(t2) - 2*q2*sin(t2) - 2*q3*sin(t2) + 2*q4*sin(t2)))/(4*RD);
	
	return result;	
}

float test_v[3];
float test_m[3];

steering_engine_torque dynamic_torque(float target_u11,float target_u22,float target_u55,float target_u1,float target_u2,float target_u5)
{
	steering_engine_torque result;
	
	static float s_c1=0.0f;                            //��Ĥ����s(t)�ڻ������ϵ��
	static float s_c2=0.0f;                            //��Ĥ����s(t)�ڻ������ϵ��
	static float s_c3=0.0f;                            //��Ĥ����s(t)�ڻ������ϵ��
	
	static float k1=1.0f;                              //S(t)��ϵ��
	static float k2=1.0f;                            
	static float k3=1.0f;                            
	
	static float error[3]={0,0,0};                    //�����  3*1������
	static float error_sum[3]={0,0,0};                //���Ļ�����
	
	float t1=Angle_Data.Angle_x*pi/180;                          //ת��Ϊ������
	float t2=Angle_Data.Angle_y*pi/180;
	float t3=Angle_Data.Angle_z*pi/180;
	
	error[0]=target_u1-reall_U1();
	error[1]=target_u2-reall_U2();
	error[2]=target_u5-reall_U5();
	
	test_v[0]=error[0];
	test_v[1]=error[1];
	test_v[2]=error[2];
	
	error_sum[0]+=error[0];
	error_sum[1]+=error[1];
	error_sum[2]+=error[2];
	if(error_sum[0]>2)error_sum[0]=2;                       //���ƻ��ֱ���
	else if(error_sum[0]<-2)error_sum[0]=-2;
	if(error_sum[1]>2)error_sum[1]=2;                       //���ƻ��ֱ���
	else if(error_sum[1]<-2)error_sum[1]=-2;
	if(error_sum[2]>2)error_sum[2]=2;                       //���ƻ��ֱ���
	else if(error_sum[2]<-2)error_sum[2]=-2;
	
	
	float M1=A11()*target_u11 + A11()*s_c1*error[0] + B_row1() + k1*(error[0]+error_sum[0]);              //���ݿ������������Ť�صľ������
	float M2=A22()*target_u22 + A22()*s_c2*error[1] + B_row2() + k2*(error[1]+error_sum[1]);
	float M3=A31()*(target_u11 + s_c1*error[0]) + A32()*(target_u22 + s_c2*error[1]) + A33()*(target_u55 + s_c3*error[2]) + B_row3() + k3*(error[2]+error_sum[2]);
	
	test_m[0]=M1;
	test_m[1]=M2;
	test_m[2]=M3;
	
	result.torque1=(M3*sin(t2))/2 - (M1*cos(t2)*cos(t3))/2 - (M2*cos(t1)*cos(t3))/2 + (M1*cos(t1)*sin(t3))/2 - (M3*cos(t2)*sin(t1))/2 - (M2*cos(t2)*sin(t3))/2 - 
	               (M1*cos(t3)*sin(t1)*sin(t2))/2 - (M2*sin(t1)*sin(t2)*sin(t3))/2 + (sqrt(2)*M3*cos(t1)*cos(t2))/4 - (sqrt(2)*M2*cos(t3)*sin(t1))/4 + (sqrt(2)*M1*sin(t1)*sin(t3))/4 + 
	               (sqrt(2)*M1*cos(t1)*cos(t3)*sin(t2))/4 + (sqrt(2)*M2*cos(t1)*sin(t2)*sin(t3))/4;
	
	if(result.torque1>10)
	{
		test_m[0]=test_m[0];
	}
	result.torque2=(M1*cos(t2)*cos(t3))/2 - (M3*sin(t2))/2 - (M2*cos(t1)*cos(t3))/2 + (M1*cos(t1)*sin(t3))/2 - (M3*cos(t2)*sin(t1))/2 + 
	               (M2*cos(t2)*sin(t3))/2 - (M1*cos(t3)*sin(t1)*sin(t2))/2 - (M2*sin(t1)*sin(t2)*sin(t3))/2 + (sqrt(2)*M3*cos(t1)*cos(t2))/4 - 
	               (sqrt(2)*M2*cos(t3)*sin(t1))/4 + (sqrt(2)*M1*sin(t1)*sin(t3))/4 + (sqrt(2)*M1*cos(t1)*cos(t3)*sin(t2))/4 + (sqrt(2)*M2*cos(t1)*sin(t2)*sin(t3))/4;
	
	result.torque3=(M1*cos(t2)*cos(t3))/2 - (M3*sin(t2))/2 + (M2*cos(t1)*cos(t3))/2 - (M1*cos(t1)*sin(t3))/2 + (M3*cos(t2)*sin(t1))/2 + 
	               (M2*cos(t2)*sin(t3))/2 + (M1*cos(t3)*sin(t1)*sin(t2))/2 + (M2*sin(t1)*sin(t2)*sin(t3))/2 + (sqrt(2)*M3*cos(t1)*cos(t2))/4 - 
	               (sqrt(2)*M2*cos(t3)*sin(t1))/4 + (sqrt(2)*M1*sin(t1)*sin(t3))/4 + (sqrt(2)*M1*cos(t1)*cos(t3)*sin(t2))/4 + (sqrt(2)*M2*cos(t1)*sin(t2)*sin(t3))/4;
	
	result.torque4=(M3*sin(t2))/2 - (M1*cos(t2)*cos(t3))/2 + (M2*cos(t1)*cos(t3))/2 - (M1*cos(t1)*sin(t3))/2 + (M3*cos(t2)*sin(t1))/2 - 
	               (M2*cos(t2)*sin(t3))/2 + (M1*cos(t3)*sin(t1)*sin(t2))/2 + (M2*sin(t1)*sin(t2)*sin(t3))/2 + (sqrt(2)*M3*cos(t1)*cos(t2))/4 - 
	               (sqrt(2)*M2*cos(t3)*sin(t1))/4 + (sqrt(2)*M1*sin(t1)*sin(t3))/4 + (sqrt(2)*M1*cos(t1)*cos(t3)*sin(t2))/4 + (sqrt(2)*M2*cos(t1)*sin(t2)*sin(t3))/4;

	return result;
}


