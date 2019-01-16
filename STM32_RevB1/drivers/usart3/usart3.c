#include "main.h"
#include "usart3.h"
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
 u8 mode_data[8];
 u8 six_data_1[4]={6,5,4,0};
 u8 six_data_2[4]={4,5,6,0};

void uart3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能 UART3 模块的时钟  使能 UART3对应的引脚端口PA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);
	//配置UART3 的发送引脚   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//配置PA9 为复用输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//刷新频率50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);    
  //配置UART3 的接收引脚 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//配置PA10为浮地输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//UART3的配置:
	USART_InitStructure.USART_BaudRate = 9600; //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;//无奇偶效验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不使用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使能发送和接收功能
	//应用配置到UART3
	USART_Init(USART3, &USART_InitStructure);
	USART_ClearFlag(USART3,USART_FLAG_TC);//避免第1个字节丢失
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //打开接收中断  
	//启动UART3
  USART_Cmd(USART3, ENABLE);
	//打开usart3中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************实现函数**********************************************
*功    能:		usart1接收一个字节
*********************************************************************************/
u8 usart3_receive(void)
{
	while((USART3->SR&0x20)==0);
	return USART3->DR;
}
/**************************实现函数**********************************************
*功    能:		usart1发送一个字节
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************实现函数**********************************************
*功    能:		usart1接收中断
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
 

