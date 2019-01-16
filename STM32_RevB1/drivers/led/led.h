#ifndef __LED_H
#define __LED_H 
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
//LED端口定义
#define LED0 PBout(8)// PB8  LED
#define LED1 PEout(2)// PE2	 蜂鸣器

#define LED2 PDout(1)     //副板LED指示灯
#define CD4 PDout(2)      //右转向灯
#define CD3 PDout(3)      //左转向灯
#define CD2 PDout(4)      //前后车灯
#define CD1 PDout(5)      //刹车灯
//#define YY6 PDout(6)

#define YY6 PGout(11)    //车灯端口设置  刹车灯
#define YY5 PGout(12)    //车灯端口设置  刹车灯
#define YY4 PGout(13)    //前后灯
#define YY3 PGout(14)    //转向灯
#define YY2 PGout(15)    //转向灯
#define YY1 PEout(0)    //车灯端口设置  刹车灯

#define YJ1 PFout(0)     //以下是液晶屏端口  大灯指示
#define YJ2 PFout(1)     //左转向等指示
#define YJ3 PFout(2)     //右转向灯指示
#define YJ4 PFout(3)     //速度
#define YJ5 PFout(4)     //模式1
#define YJ6 PFout(5)     //模式2
#define YJ7 PFout(6)     //模式3
#define YJ8 PFout(7)     //模式4

void led_init(void);       //引脚配置函数
void Led_Flash(u16 time);  //led运算翻转函数
void speed(int values);    //速度计算函数
#endif

