#ifndef __FILTER_H
#define __FILTER_H

/**************************************************************************
���ߣ���������
��汾��V3.5
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
