#ifndef __KEY_H
#define __KEY_H	 
#include "main.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
//#define KEY1 PBin(5)

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)//��ȡ����1
#define KEY2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)//��ȡ����2 
#define KEY3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)//��ȡ����3   
//������̤�忪�س�ʼ

//#define KEY7  GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)//��ȡ����0
//#define KEY6 	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)//��ȡ����1
//#define KEY5  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9)//��ȡ����2 
//#define KEY4  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)//��ȡ����3 
//������ң�����˿ڳ�ʼ

void KEY_Init(void);
void key(void);
u8 KEY_Scan(u8 mode);
#endif
