/**************************************************************************
���ߣ���������
��汾��V3.5
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
/*����printf�ر�����*/
/************************************************************/
int fputc(int ch, FILE *f)
{
	USART1->DR=(u8)ch;
	while((USART1->SR&0X40)==0);
	return ch;
}
/************************************************************/
/**************************ʵ�ֺ���**********************************************
*��    ��:		����usart1
*���������     ������
*********************************************************************************/
void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//����UART1 �ķ�������   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����PA9 Ϊ�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//ˢ��Ƶ��50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  //����UART1 �Ľ������� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//����PA10Ϊ��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UART1������:
	USART_InitStructure.USART_BaudRate = 115200; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No ;//����żЧ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��ʹ��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ʹ�ܷ��ͺͽ��չ���
	//Ӧ�����õ�UART1
	USART_Init(USART1, &USART_InitStructure);
	USART_ClearFlag(USART1,USART_FLAG_TC);//�����1���ֽڶ�ʧ
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�򿪽����ж�  
	//����UART1
  USART_Cmd(USART1, ENABLE);
	//��usart1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
u8 usart1_receive(void)
{
	while((USART1->SR&0x20)==0);
	return USART1->DR;
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
void usart1_send(u8 data)
{
	while (!(USART1->SR & USART_FLAG_TXE));
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1�����ж�
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
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��1
*********************************************************************************/
void command1(void)
{
	 
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��2
*********************************************************************************/
void command2(void)
{
	 
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1ָ��3
*********************************************************************************/
void command3(void)
{
	 
}
/**************************�������*********************************************/
