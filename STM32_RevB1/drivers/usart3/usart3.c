#include "main.h"
#include "usart3.h"
/**************************************************************************
���ߣ���������
��汾��V3.5
**************************************************************************/
 u8 mode_data[8];
 u8 six_data_1[4]={6,5,4,0};
 u8 six_data_2[4]={4,5,6,0};

void uart3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ�� UART3 ģ���ʱ��  ʹ�� UART3��Ӧ�����Ŷ˿�PA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);
	//����UART3 �ķ�������   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����PA9 Ϊ�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//ˢ��Ƶ��50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);    
  //����UART3 �Ľ������� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//����PA10Ϊ��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//UART3������:
	USART_InitStructure.USART_BaudRate = 9600; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No ;//����żЧ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��ʹ��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ʹ�ܷ��ͺͽ��չ���
	//Ӧ�����õ�UART3
	USART_Init(USART3, &USART_InitStructure);
	USART_ClearFlag(USART3,USART_FLAG_TC);//�����1���ֽڶ�ʧ
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //�򿪽����ж�  
	//����UART3
  USART_Cmd(USART3, ENABLE);
	//��usart3�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
u8 usart3_receive(void)
{
	while((USART3->SR&0x20)==0);
	return USART3->DR;
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1�����ж�
*********************************************************************************/
void UART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  	{
		u8 rxTemp;
		rxTemp=usart3_receive();
		if(rxTemp==0xa5)
		{
			rxTemp=usart3_receive(); 
			if(rxTemp==0x5a)
			{
				switch (usart3_receive())
				{
					case 0xa1:	command1();
					      		break;
					case 0xa2:	command2();
					      		break;
					case 0xa3:	command3();
					      		break;
					//default:	usart1_send(0xff);
				} 
			}
		}
	}
}
 

