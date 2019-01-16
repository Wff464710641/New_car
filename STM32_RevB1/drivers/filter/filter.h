#ifndef __FILTER_H
#define __FILTER_H

/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
