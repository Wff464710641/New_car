#ifndef __MINIBALANCE_H
#define __MINIBALANCE_H
#include "main.h"
#include "filter.h"
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/

//extern float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;



extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void);  
int velocity_left(int encoder_left);
int velocity_right(int encoder_right);
void Set_Pwm(int moto1,int moto2);

void readEncoder(void);

void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);

#endif
