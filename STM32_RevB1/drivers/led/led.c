
#include "main.h"
#include "key.h"
/**************************************************************************
���ߣ���������
��汾��V3.5
**************************************************************************/
/**************************ʵ�ֺ���**********************************************
*��    ��:		����led����
*********************************************************************************/
void led_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
 //����LED1�˿ڳ�ʼ	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;				 //LED0-->PC.2 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
 GPIO_ResetBits(GPIOC,GPIO_Pin_2);						 //PB.8 ����͵�ƽ
//�������˿ڳ�ʼ  ��������GPIOE3
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //LED0-->PC.2 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
 GPIO_ResetBits(GPIOE,GPIO_Pin_3);	
}
/**************************�������*********************************************/

/**************************************************************************
�������ܣ��ٶȼ��㺯��
��ڲ���������������ֵ
����  ֵ����
���ߣ���������
��汾��V3.5
�������������ñ��������ص�ֵ���㵱ǰ�ٶ�
**************************************************************************/

void speed(int values)
{
	static int add = 0;  //����һ���ۼ���ʱ����
	static int addi=0;
   if(values<20&&values>0) addi = 100; 
	if(values<190&&values>140) addi = 15; 
	if(values<360&&values>310) addi = 8; 
	if(values<470&&values>420) addi = 6; 
	if(values<690&&values>630) addi = 4; 
	else
	{
		addi = (690-values)/10;
	}
	if(++add >= addi)     //5ms���뺯���Ӽ�һ��  ����������ֵ
	{	
     add = 0;		 //�ۼ�ֵ���¸�ֵ
	  YJ4=!YJ4;     //Һ���������źŷ�ת  ʹ���ٶ���ʾ
	}
}



