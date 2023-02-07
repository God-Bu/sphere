#ifndef __KINEMATIC_H
#define __KINEMATIC_H

#include <stm32f4xx.h>

int16_t kinematic_ID1(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ);
int16_t kinematic_ID2(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ);
int16_t kinematic_ID3(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ);
int16_t kinematic_ID4(float Angle_x,float Angle_y,float Angle_z,float VX,float VY,float WZ);



#endif
