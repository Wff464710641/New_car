#ifndef __LED_H
#define __LED_H 
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
//LED�˿ڶ���
#define LED0 PBout(8)// PB8  LED
#define LED1 PEout(2)// PE2	 ������

#define LED2 PDout(1)     //����LEDָʾ��
#define CD4 PDout(2)      //��ת���
#define CD3 PDout(3)      //��ת���
#define CD2 PDout(4)      //ǰ�󳵵�
#define CD1 PDout(5)      //ɲ����
//#define YY6 PDout(6)

#define YY6 PGout(11)    //���ƶ˿�����  ɲ����
#define YY5 PGout(12)    //���ƶ˿�����  ɲ����
#define YY4 PGout(13)    //ǰ���
#define YY3 PGout(14)    //ת���
#define YY2 PGout(15)    //ת���
#define YY1 PEout(0)    //���ƶ˿�����  ɲ����

#define YJ1 PFout(0)     //������Һ�����˿�  ���ָʾ
#define YJ2 PFout(1)     //��ת���ָʾ
#define YJ3 PFout(2)     //��ת���ָʾ
#define YJ4 PFout(3)     //�ٶ�
#define YJ5 PFout(4)     //ģʽ1
#define YJ6 PFout(5)     //ģʽ2
#define YJ7 PFout(6)     //ģʽ3
#define YJ8 PFout(7)     //ģʽ4

void led_init(void);       //�������ú���
void Led_Flash(u16 time);  //led���㷭ת����
void speed(int values);    //�ٶȼ��㺯��
#endif

