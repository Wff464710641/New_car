/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
#include "main.h"
u8 rxTemp=0;
u8 machine_state = STATE_IDLE;
u8 rx_buf_tmp=0,rx_parity=0,syn_state=FALSE,frame_seq=0;
u8 cmd_length=0;
u8 payload_num=0;
u8 payload[MAX_PAYLOAD_NUM];
u8 rx_buff[RX_BUFF_MAX];
u8 ptr_start=0,ptr_end=0;


uint8_t buffer1[14];
/************************************************************/
/*串口printf必备程序*/
/************************************************************/
int fputc(int ch, FILE *f)
{
	USART1->DR=(u8)ch;
	while((USART1->SR&0X40)==0);
	return ch;
}
/************************************************************/
/**************************实现函数**********************************************
*功    能:		定义usart1
*输入参数：     波特率
*********************************************************************************/
void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//配置UART1 的发送引脚   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//配置PA9 为复用输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//刷新频率50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  //配置UART1 的接收引脚 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//配置PA10为浮地输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART1的配置:
	USART_InitStructure.USART_BaudRate = 115200; //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;//无奇偶效验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不使用硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使能发送和接收功能
	//应用配置到UART1
	USART_Init(USART1, &USART_InitStructure);
	USART_ClearFlag(USART1,USART_FLAG_TC);//避免第1个字节丢失
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //打开接收中断  
	//启动UART1
  USART_Cmd(USART1, ENABLE);
	//打开usart1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************实现函数**********************************************
*功    能:		usart1接收一个字节
*********************************************************************************/
u8 usart1_receive(void)
{
	while((USART1->SR&0x20)==0);
	return USART1->DR;
}
/**************************实现函数**********************************************
*功    能:		usart1发送一个字节
*********************************************************************************/
void usart1_send(u8 data)
{
	while (!(USART1->SR & USART_FLAG_TXE));
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************实现函数**********************************************
*功    能:		usart1接收中断
*********************************************************************************/
void USART1_IRQHandler(void)
{
///*
	u8 tmp_reg=0;
	tmp_reg = USART1->SR;
	if(((tmp_reg&0x20)==0) && (tmp_reg&0x08) != 0)		// fix bug: rxne =0 but overrun bit =1 
	{
		tmp_reg = USART1->SR;
		tmp_reg = USART1->DR;
	}
	tmp_reg = USART1->CR1;
	tmp_reg = USART1->CR2;
	tmp_reg = USART1->CR3;

//	rx_buf_tmp = USART1->DR;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rx_buff[ptr_end] = usart1_receive();
		ptr_end++;
		if(ptr_end >= RX_BUFF_MAX)
		{
			ptr_end = 0;
		}
		
//		usart1_send(rx_buf_tmp);
/*
		switch(machine_state)
		{
			case STATE_IDLE:
				if(FRAME_HEADER1 == rx_buf_tmp)
				{
					machine_state = STATE_RX_HDR1;
					rx_parity = rx_parity ^ rx_buf_tmp;
				}
				else
				{
//				usart1_send(0x01);
//				usart1_send(rx_buf_tmp);
					syn_state = FALSE;
					machine_state = STATE_IDLE;
					rx_parity = 0;
				}
			break;
					
			case STATE_RX_HDR1:
				if(FRAME_HEADER2 == rx_buf_tmp)
				{
					machine_state = STATE_RX_HDR2;
					rx_parity = rx_parity ^ rx_buf_tmp;
				}
				else
				{
//				usart1_send(0x02);
//				usart1_send(rx_buf_tmp);
					syn_state = FALSE;
					machine_state = STATE_IDLE;
					rx_parity = 0;
				}
			break;

			case STATE_RX_HDR2:
				if((CMD_SPEED_LENGTH == rx_buf_tmp) || (CMD_BATT_LENGTH == rx_buf_tmp) 
				|| (CMD_REMAIN_DIST_LENGTH == rx_buf_tmp) ||(CMD_BATT_TEMP_LENGTH == rx_buf_tmp))
				{
					machine_state = STATE_RX_LENGTH;
					cmd_length = rx_buf_tmp;
					rx_parity = rx_parity ^ rx_buf_tmp;
				}
				else
				{
//				usart1_send(0x03);
//				usart1_send(rx_buf_tmp);
					syn_state = FALSE;
					machine_state = STATE_IDLE;
					rx_parity = 0;
				}
			break;

			case STATE_RX_LENGTH:
//			if(0 != rx_buf_tmp)
				machine_state = STATE_RX_SEQ;
				frame_seq = rx_buf_tmp;
				rx_parity = rx_parity ^ rx_buf_tmp;
			break;

			case STATE_RX_SEQ:
				if(payload_num<cmd_length)
				{
					machine_state = STATE_RX_SEQ;	
					payload[payload_num] = rx_buf_tmp;
					rx_parity = rx_parity ^ rx_buf_tmp;
				}
				payload_num++;
				if(payload_num == cmd_length)
				{
					payload_num = 0;
					machine_state = STATE_RX_PAYLOAD;
				}
			break;
				
			case STATE_RX_PAYLOAD:
				if(rx_parity == rx_buf_tmp)
				{
					machine_state = STATE_IDLE;
					syn_state = TRUE;
					rx_parity = 0;
				}
				else
				{
//				usart1_send(0x04);
//				usart1_send(rx_buf_tmp);
					syn_state = FALSE;
					machine_state = STATE_IDLE;
					rx_parity = 0;
				}
			break;

			default:
				machine_state = STATE_IDLE;
				rx_parity = 0;
				syn_state = FALSE;
			break;
		}
*/
	}
//*/	
}
/**************************实现函数**********************************************
*功    能:		usart1指令1
*********************************************************************************/
void command1(void)
{
	 
}
/**************************实现函数**********************************************
*功    能:		usart1指令2
*********************************************************************************/
void command2(void)
{
	 
}
/**************************实现函数**********************************************
*功    能:		usart1指令3
*********************************************************************************/
void command3(void)
{
	 
}
/**************************程序结束*********************************************/
