#include "key.h"
//#include "MiniBalance.h"
/**************************************************************************
���ߣ���������
��汾��V3.5
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//��ʼ��KEY0-->GPIOA.13,KEY1-->GPIOA.15  ��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//ʹ��PORTA,PORTEʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_11;//PING5-8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOG5 6 7 8
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;//PING5-8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOG5 6 7 8
} 
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ����
��    �ߣ���������
**************************************************************************/
void key(void)
{
	static u8 i=0; //��ʱ��ʱ����
   	if(temp1==1)   //�������������
	{
			if(KEY0==0||KEY1==0||KEY2==0||KEY3==0||temp4==0)  //���̤�忪�ذ��� ���������� 
			{
				if(tabanyy1==0) YY1=0; //���ϳ�����
				 
				if (temp5>=3)   //��ʱ
				{
					
					temp9=1;
					Flag_Stop=0;  //����
					
					tabanyy1=1;    //�ر��ϳ���������
					if(++i>5) i=0,YY1=1;   //���߱�������
				}
				
				if(KEY0==0||KEY1==0||KEY2==0||KEY3==0) temp4=1;  //����������ʱ ̤�忪�ذ��� ���������Զ����
			}
			else  //���̤�忪��û�а��� ����������
			{
				    
				    tabanyy1=0;   //�����ϳ���������ֵ
				    YY1=1;  //����YY1 
				    temp9=0;
					Flag_Stop=1;	//�ر�
			}
    }
	else  //���ң�ػ�������û������  ����������
	{
		   
  	  		temp9=0;
 			Flag_Stop=1;	//�ر�
	}

}









