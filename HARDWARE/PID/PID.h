#ifndef __PID_H
#define __PID_H
#include "sys.h"
extern int Encoder_Wheel;          //±àÂëÆ÷µÄÂö³å¼ÆÊı
void control(void);
int Vertical_PD(float tar_Med,float rt_Angle,float gyro_x);
int Velocity_PI(int encoder_motor);
int Velocity_IncCtrl_PI (int Encoder1,int Target1);
#endif
