#include "kinematic.h"
#include "math.h"

#define pi 3.1415926                  //基本参数
#define R 150
//#define R_in 77
#define r 41
#define d 3
#define RD (R-d)
//#define q 30*pi/180


float aa,bb,cc,dd,ee;
int16_t kinematic_ID1(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ)       //第一个舵机的速度   输入为目标速度 mm/s
{
	static int16_t velocity_ID1;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	float A11=-(RD*(sin(t3) + cos(t2)*cos(t3) - sqrt(2)*sin(t2)*sin(t3)))/(2*R*r*cos(t2));
	float A12=-(RD*(cos(t2)*sin(t3) - cos(t3) + sqrt(2)*cos(t3)*sin(t2)))/(2*R*r*cos(t2));
	float A13=(sqrt(2)*RD)/(2*r);
//	aa=A12;
//	bb=(2*R*r*cos(t2));
//	cc=-(RD*(cos(t2)*sin(t3) - cos(t3) + sqrt(2)*cos(t3)*sin(t2)));
//	dd=cos(t2)*sin(t3) - cos(t3) + sqrt(2)*cos(t3)*sin(t2);
//	ee=-(RD*-10.0f);
	velocity_ID1=-(A11*VX+A12*VY+A13*WZ)*2048/pi;
	if(velocity_ID1>3500)velocity_ID1=3500;
	if(velocity_ID1<-3500)velocity_ID1=-3500;
	return velocity_ID1;
}

int16_t kinematic_ID2(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ)       //第2个舵机的速度   输入为目标速度 mm/s
{
	static int16_t velocity_ID2;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	float A21=(RD*(sin(t3) - cos(t2)*cos(t3) + sqrt(2)*sin(t2)*sin(t3)))/(2*R*r*cos(t2));
	float A22=-(RD*(cos(t3) + cos(t2)*sin(t3) + sqrt(2)*cos(t3)*sin(t2)))/(2*R*r*cos(t2));
	float A23=(sqrt(2)*RD)/(2*r);
	velocity_ID2=-(A21*VX+A22*VY+A23*WZ)*2048/pi;
	if(velocity_ID2>3500)velocity_ID2=3500;
	if(velocity_ID2<-3500)velocity_ID2=-3500;
	return velocity_ID2;
}

int16_t kinematic_ID3(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ)       //第3个舵机的速度   输入为目标速度 mm/s
{
	static int16_t velocity_ID3;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	float A31=(RD*(sin(t3) + cos(t2)*cos(t3) + sqrt(2)*sin(t2)*sin(t3)))/(2*R*r*cos(t2));
	float A32=-(RD*(cos(t3) - cos(t2)*sin(t3) + sqrt(2)*cos(t3)*sin(t2)))/(2*R*r*cos(t2));
	float A33=(sqrt(2)*RD)/(2*r);
	velocity_ID3=-(A31*VX+A32*VY+A33*WZ)*2048/pi;
	if(velocity_ID3>3500)velocity_ID3=3500;
	if(velocity_ID3<-3500)velocity_ID3=-3500;
	return velocity_ID3;
}

int16_t kinematic_ID4(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ)       //第4个舵机的速度   输入为目标速度 mm/s
{
	static int16_t velocity_ID4;
	float t1=Angle_x*pi/180;
	float t2=Angle_y*pi/180;
	float t3=Angle_z*pi/180;
	float A41=(RD*(cos(t2)*cos(t3) - sin(t3) + sqrt(2)*sin(t2)*sin(t3)))/(2*R*r*cos(t2));
	float A42=(RD*(cos(t3) + cos(t2)*sin(t3) - sqrt(2)*cos(t3)*sin(t2)))/(2*R*r*cos(t2));
	float A43=(sqrt(2)*RD)/(2*r);
	velocity_ID4=-(A41*VX+A42*VY+A43*WZ)*2048/pi;
	if(velocity_ID4>3500)velocity_ID4=3500;
	if(velocity_ID4<-3500)velocity_ID4=-3500;
	return velocity_ID4;
}




