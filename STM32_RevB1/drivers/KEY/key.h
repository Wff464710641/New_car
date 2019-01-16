#ifndef __KEY_H
#define __KEY_H	 
#include "main.h"
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
//#define KEY1 PBin(5)

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)//读取按键1
#define KEY2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)//读取按键2 
#define KEY3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)//读取按键3   
//以上是踏板开关初始

//#define KEY7  GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)//读取按键0
//#define KEY6 	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)//读取按键1
//#define KEY5  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9)//读取按键2 
//#define KEY4  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)//读取按键3 
//以上是遥控器端口初始

void KEY_Init(void);
void key(void);
u8 KEY_Scan(u8 mode);
#endif
