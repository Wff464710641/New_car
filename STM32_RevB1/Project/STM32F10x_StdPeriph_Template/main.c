#include "main.h"
#include "filter.h"
#include "key.h"
#include "iwdg.h"

#define	DBG_ADC 0
#define	debug_selftest 0 
#define	speed_diff_enable 1
#define	HARDWARE_VER_RevA1_1
#define	HARDWARE_VER	0x11
#define	FIRMWARE_VER	0x12


//Firmware version: 1.02	增加版本号读取功能，增加清华定制，120转电机支持，增加PM1支持，整理代码，改成函数，增加电机不分左右的支持
//Firmware version: 1.01	


short	pwm_f_l=0,pwm_f_r=0;
short	pwm_e_l=0,pwm_e_r=0;
short pwm_diff_speed=0;
u8 ps2_data = 0;

u16 main_current=0;        //过流保护变量

 
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 （有的6050使用DMP时，需要开机后不停摇晃小车10S左右，等待数据稳定）
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=1;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
long encoder_left_sigma=0,encoder_right_sigma=0;		//left & right encoder sigma
u8 encoder_rst_flag=0;
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
u16 Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据

int qian = 0;
int hou  = 0;

u8 error_flag_num=0;
u8 state_normal = FALSE;
u8 rcv_whole_packet = FALSE;
u8 rst_state = FALSE;
u8 counter_5ms = 0;
u16 counter_buzz_5ms = 0;
u8 led_flash = 0;
u8 counter_accident=0;
u8 tx_encoder_buff[MAX_TX_PAYLOAD_NUM];


u8 psx_transfer(u8 dat) 
{
	unsigned char rd_data ,wt_data, i;
	wt_data = dat;
	rd_data = 0;
	for(i = 0;i < 8;i++)
	{

//		delay_ms(1);
		PS2_CLK_1;
		if(wt_data & (0x01 << i))
		{
			PS2_CMD_1;
		}
		else
		{
			PS2_CMD_0;
		}
		delay_us(10);
		
		PS2_CLK_0;
		delay_us(8);
		if(PS2_DAT != 0) 
		{
			rd_data |= (0x01<<i);
		}
		delay_us(2);
		PS2_CLK_1;
//		delay_ms(1);
	}
	return rd_data;
}

void psx_write_read(u8 *ps2_get_buf) {
	PS2_ATT_0;
	delay_ms(1);
	ps2_get_buf[0] = psx_transfer(START_CMD);
	ps2_get_buf[1] = psx_transfer(ASK_DAT_CMD);
	ps2_get_buf[2] = psx_transfer(ps2_get_buf[0]);
	ps2_get_buf[3] = psx_transfer(ps2_get_buf[0]);
	ps2_get_buf[4] = psx_transfer(ps2_get_buf[0]);
	if(ps2_get_buf[1] == PS2_LED_RED)
	{
		ps2_get_buf[5] = psx_transfer(ps2_get_buf[0]);
		ps2_get_buf[6] = psx_transfer(ps2_get_buf[0]);
		ps2_get_buf[7] = psx_transfer(ps2_get_buf[0]);
		ps2_get_buf[8] = psx_transfer(ps2_get_buf[0]);
	}
	delay_ms(1);	
	PS2_ATT_1
	return;
}

void ps2_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);	 //使能PB,PE端口时钟
	//主板LED1端口初始	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;				 //LED0-->PC.2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 	//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 			//
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //LED0-->PC.2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 	//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 			//
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PC.2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 	//
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 			//
	
}


/**************************************************************************
函数功能：主函数 初始化系统和外设
作者：流星落泪
库版本：V3.5
**************************************************************************/
int main(void)
{
//	u8 machine_state = STATE_IDLE;
//	u8 rx_buf_tmp=0,rx_parity=0,syn_state=FALSE,frame_seq=0;
	short pwm_f_l_preset=0,pwm_f_r_preset=0,pwm_e_l_preset=0,pwm_e_r_preset=0;
	int speed_percent=25,turn_percent=25;
	short pwm_f_l_speed=0,pwm_f_l_turn=0,pwm_f_r_speed=0,pwm_f_r_turn=0;
	u8 psx_buf[PS2_MAX_BUF_SIZE],psx_buf_bak[PS2_MAX_BUF_SIZE];
	u8 flag = 0;
//	u8 payload[MAX_PAYLOAD_NUM];
	u8 tx_buff[MAX_TX_PAYLOAD_NUM];
//	u8 tmp_reg=0;

//	u8 cmd_length=0;
//	u8 payload_num=0;
	u16 i=0;
	SystemInit();                   //=====系统初始化
	delay_init(72);                 //=====延时函数
	usart1_init();                  //=====串口1初始化 波特率：115200
	uart2_init();            				//=====串口2初始化 波特率：115200
//	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
//	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	led_init();                     //=====gpio output init
	ps2_init();
	KEY_Init();                     //=====gpio input init
	Adc_Init();	                    //=====初始化ADC模块
	MiniBalance_PWM_Init(1999,0);   //=====初始化PWM 36KHZ 高频可以防止电机低频时的尖叫声
	//OLED_Init();	                //=====初始化OLED 模拟SPI 
	Encoder_Init();                 //=====初始化编码器1
	Encoder_Init2();	            	//=====初始化编码器2
	delay_ms(50);                  	//=====延时等待稳定		
	usart2_send(0xaa);
#ifdef TIMER_5MS
	Timer1_Init(49,7199);           //5MS进一次中断服务函数 中断服务函数在minibalance.c里面
#elif defined TIMER_100MS
	Timer1_Init(199,7199);           //100MS进一次中断服务函数 中断服务函数在minibalance.c里面
#endif

//	IWDG_Init() ;//独立看门狗
	payload_num = 0;
	i=0;
	if(KEY1==0)
	state_normal = TRUE;
	BUZZER_ON;
	delay_ms(200); 
	BUZZER_OFF;
	delay_ms(200); 
	BUZZER_ON;
	delay_ms(200); 
	BUZZER_OFF;
	delay_ms(200); 
	
//	printf("xxxxxxxxxxxxxxxxxxxx\n");
	while(1)
	{
//		IWDG_ReloadCounter();
//		printf("24V = %x%x\n",(Voltage<<8),Voltage);
//		printf("Cur = %x%x\n",(main_current<<8),main_current);
//		delay_ms(500); 
		if(debug_selftest)
		{
			pwm_f_l = 0;
			pwm_f_r = i;
			pwm_e_l = 0;
			pwm_e_r = 0;
			if(flag == 0)
			i+=25;
			else
			i-=25;
			
			if(i>250)
			{
//				i=0;
				flag = 1;
			}
			if(i<5)
			{
//				i=0;
				flag = 0;
			}
			if(1)
			{
			usart1_send(0x55);
			usart1_send(0xaa);
			usart1_send(((Voltage & 0xff00)>>8));
			usart1_send(((Voltage & 0x00ff)>>0));
			usart1_send(((main_current & 0xff00)>>8));
			usart1_send(((main_current & 0x00ff)>>0));
			}
			delay_ms(200);
			delay_ms(200);
			delay_ms(200);
			delay_ms(200);
			delay_ms(200);
//			delay_ms(200);
//			delay_ms(200);
//			delay_ms(200);
//			delay_ms(200);
//			delay_ms(200);
		}
		
/*
		tmp_reg = USART1->SR;
		if(((tmp_reg&0x20)==0) && (tmp_reg&0x08) != 0)		// fix bug: rxne =0 but overrun bit =1 
		{
			tmp_reg = USART1->SR;
			tmp_reg = USART1->DR;
			usart1_send(0x55);
		}
		if((tmp_reg&0x08) != 0)														// overrun bit =1 
		{
			tmp_reg = USART1->SR;
			tmp_reg = USART1->DR;
			usart1_send(0xaa);
		}
*/
		if(KEY1==1)
		{
			if(ptr_end != ptr_start)
			{
				rx_buf_tmp = rx_buff[ptr_start];
				ptr_start++;
				if(ptr_start>=RX_BUFF_MAX)
				{
					ptr_start = 0;
				}
//			usart1_send(rx_buf_tmp);
//			delay_ms(200);
//			printf("aa=%d\n",rx_buf_tmp);
//			delay_ms(200);
//			printf("m=%d\n",machine_state);
//			printf("p=%d\n",payload_num);
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
							syn_state = FALSE;
							machine_state = STATE_IDLE;
							rx_parity = 0;
						}
					break;

					case STATE_RX_LENGTH:
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
	//*/			
				if(syn_state == TRUE)
				{
					counter_5ms = 0;
					tx_buff[0] = TX_FRAME_HDR1;
					tx_buff[1] = TX_FRAME_HDR2;
					tx_buff[3] = frame_seq;

					if(payload[0] == SPEED_CMD)
					{
						tx_buff[2] = CMD_SPEED_LENGTH;
						if(state_normal == TRUE)
						{	
							tx_buff[4] = SPEED_CMD;
							tx_buff[5] = tx_encoder_buff[5];
							tx_buff[6] = tx_encoder_buff[6];
							tx_buff[7] = tx_encoder_buff[7];
							tx_buff[8] = tx_encoder_buff[8];
							tx_buff[9] = 0;
							tx_buff[10] = 0;
							tx_buff[11] = 0;
							tx_buff[12] = 0;
							tx_buff[13] = 0;
							if(rst_state == TRUE)
							{
								pwm_f_l_preset = payload[1]*256 + payload[2];
								pwm_f_r_preset = payload[3]*256 + payload[4];
								pwm_e_l_preset = payload[5]*256 + payload[6];
								pwm_e_r_preset = payload[7]*256 + payload[8];
								usart2_send(0xaa);
								usart2_send(payload[1]);
								usart2_send(payload[2]);
								usart2_send(payload[3]);
								usart2_send(payload[4]);
								
								if((pwm_f_l_preset>0)&&(pwm_f_l_preset>MAX_SPEED))
								{
									pwm_f_l_preset = MAX_SPEED;
								}
								if((pwm_f_l_preset<0)&&(pwm_f_l_preset<-MAX_SPEED))
								{
									pwm_f_l_preset = -MAX_SPEED;
								}

								if((pwm_f_r_preset>0)&&(pwm_f_r_preset>MAX_SPEED))
								{
									pwm_f_r_preset = MAX_SPEED;
								}
								if((pwm_f_r_preset<0)&&(pwm_f_r_preset<-MAX_SPEED))
								{
									pwm_f_r_preset = -MAX_SPEED;
								}

								if((pwm_e_l_preset>0)&&(pwm_e_l_preset>MAX_SPEED))
								{
									pwm_e_l_preset = MAX_SPEED;
								}
								if((pwm_e_l_preset<0)&&(pwm_e_l_preset<-MAX_SPEED))
								{
									pwm_e_l_preset = -MAX_SPEED;
								}

								if((pwm_e_r_preset>0)&&(pwm_e_r_preset>MAX_SPEED))
								{
									pwm_e_r_preset = MAX_SPEED;
								}
								if((pwm_e_r_preset<0)&&(pwm_e_r_preset<-MAX_SPEED))
								{
									pwm_e_r_preset = -MAX_SPEED;
								}

								if(pwm_f_l>0)
								{
									if(pwm_f_l_preset < 0)
									{
										pwm_f_l = 0;
									}
									else
									{
										if((pwm_f_l_preset-pwm_f_l) > MAX_STEP)
										{	
											pwm_f_l += MAX_STEP;
										}
										else
										{
											pwm_f_l = pwm_f_l_preset;
										}
									}
								}
								else if(pwm_f_l==0)
								{
									if((pwm_f_l - pwm_f_l_preset) > MAX_STEP)
									{	
										pwm_f_l -= MAX_STEP;
									}
									else if((pwm_f_l_preset - pwm_f_l) > MAX_STEP)
									{
										pwm_f_l += MAX_STEP;
									}
									else
									{
										pwm_f_l = pwm_f_l_preset;
									}
								}
								else 
								{
									if(pwm_f_l_preset > 0)
									{
										pwm_f_l = 0;
									}
									else
									{
										if((pwm_f_l - pwm_f_l_preset) > MAX_STEP)
										{	
											pwm_f_l -= MAX_STEP;
										}
										else
										{
											pwm_f_l = pwm_f_l_preset;
										}
									}
								}	
							
								if(pwm_f_r>0)
								{
									if(pwm_f_r_preset < 0)
									{
										pwm_f_r = 0;
									}
									else
									{
										if((pwm_f_r_preset-pwm_f_r) > MAX_STEP)
										{	
											pwm_f_r += MAX_STEP;
										}
										else
										{
											pwm_f_r = pwm_f_r_preset;
										}
									}
								}
								else if(pwm_f_r==0)
								{
									if((pwm_f_r - pwm_f_r_preset) > MAX_STEP)
									{	
										pwm_f_r -= MAX_STEP;
									}
									else if((pwm_f_r_preset - pwm_f_r) > MAX_STEP)
									{
										pwm_f_r += MAX_STEP;
									}
									else
									{
										pwm_f_r = pwm_f_r_preset;
									}
								}
								else 
								{
									if(pwm_f_r_preset > 0)
									{
										pwm_f_r = 0;
									}
									else
									{
										if((pwm_f_r - pwm_f_r_preset) > MAX_STEP)
										{	
											pwm_f_r -= MAX_STEP;
										}
										else
										{
											pwm_f_r = pwm_f_r_preset;
										}
									}
								}							

								if(pwm_e_l>0)
								{
									if(pwm_e_l_preset < 0)
									{
										pwm_e_l = 0;
									}
									else
									{
										if((pwm_e_l_preset-pwm_e_l) > MAX_STEP)
										{	
											pwm_e_l += MAX_STEP;
										}
										else
										{
											pwm_e_l = pwm_e_l_preset;
										}
									}
								}
								else if(pwm_e_l==0)
								{
									if((pwm_e_l - pwm_e_l_preset) > MAX_STEP)
									{	
										pwm_e_l -= MAX_STEP;
									}
									else if((pwm_e_l_preset - pwm_e_l) > MAX_STEP)
									{
										pwm_e_l += MAX_STEP;
									}
									else
									{
										pwm_e_l = pwm_e_l_preset;
									}
								}
								else 
								{
									if(pwm_e_l_preset > 0)
									{
										pwm_e_l = 0;
									}
									else
									{
										if((pwm_e_l - pwm_e_l_preset) > MAX_STEP)
										{	
											pwm_e_l -= MAX_STEP;
										}
										else
										{
											pwm_e_l = pwm_e_l_preset;
										}
									}
								}							

								if(pwm_e_r>0)
								{
									if(pwm_e_r_preset < 0)
									{
										pwm_e_r = 0;
									}
									else
									{
										if((pwm_e_r_preset-pwm_e_r) > MAX_STEP)
										{	
											pwm_e_r += MAX_STEP;
										}
										else
										{
											pwm_e_r = pwm_e_r_preset;
										}
									}
								}
								else if(pwm_e_r==0)
								{
									if((pwm_e_r - pwm_e_r_preset) > MAX_STEP)
									{	
										pwm_e_r -= MAX_STEP;
									}
									else if((pwm_e_r_preset - pwm_e_r) > MAX_STEP)
									{
										pwm_e_r += MAX_STEP;
									}
									else
									{
										pwm_e_r = pwm_e_r_preset;
									}
								}
								else 
								{
									if(pwm_e_r_preset > 0)
									{
										pwm_e_r = 0;
									}
									else
									{
										if((pwm_e_r - pwm_e_r_preset) > MAX_STEP)
										{	
											pwm_e_r -= MAX_STEP;
										}
										else
										{
											pwm_e_r = pwm_e_r_preset;
										}
									}
								}
								if(speed_diff_enable)
								{
									if((pwm_f_l - pwm_f_r) > MAX_DIFF_SPEED)
									{
										pwm_diff_speed = pwm_f_l - pwm_f_r;
										pwm_f_l = pwm_f_l * MAX_DIFF_SPEED / pwm_diff_speed;
										pwm_f_r = pwm_f_r * MAX_DIFF_SPEED / pwm_diff_speed;
									}
									else if((pwm_f_r - pwm_f_l) > MAX_DIFF_SPEED)
									{
										pwm_diff_speed = pwm_f_r - pwm_f_l;
										pwm_f_l = pwm_f_l * MAX_DIFF_SPEED / pwm_diff_speed;
										pwm_f_r = pwm_f_r * MAX_DIFF_SPEED / pwm_diff_speed;
									}
								}
							}							
							else
							{
								pwm_f_l = 0;
								pwm_f_r = 0;
								pwm_e_l = 0;
								pwm_e_r = 0;
							}
							for(i=0;i<(CMD_SPEED_LENGTH+4);i++)
							{
								tx_buff[13] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_SPEED_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
					else if(payload[0] == BATT_CMD)
					{
						tx_buff[2] = CMD_BATT_LENGTH;
						if(state_normal == TRUE)
						{	
							tx_buff[4] = BATT_CMD;
							if(Voltage > MIN_VOLTAGE)
							{
								tx_buff[5] = 100*(Voltage - MIN_VOLTAGE)/(MAX_VOLTAGE-MIN_VOLTAGE);
							}
							else
							{
								tx_buff[5] = 0;
							}
							tx_buff[6] = 0;
							for(i=0;i<(CMD_BATT_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_BATT_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;						
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
					else if(payload[0] == REMAIN_DIST_CMD)
					{
						tx_buff[2] = CMD_REMAIN_DIST_LENGTH;
						if(state_normal == TRUE)
						{	
							tx_buff[4] = REMAIN_DIST_CMD;
							tx_buff[5] = MAX_DIST*(Voltage - MIN_VOLTAGE)/MAX_VOLTAGE;
							tx_buff[6] = 0;
							for(i=0;i<(CMD_REMAIN_DIST_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_REMAIN_DIST_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;						
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
#ifdef	HARDWARE_VER_RevA1_1
					else if(payload[0] == MAIN_CURRENT_CMD)
					{
						tx_buff[2] = CMD_MAIN_CURRENT_LENGTH;
						if(state_normal == TRUE)
						{	
							tx_buff[4] = MAIN_CURRENT_CMD;
							if(main_current > ZERO_CURRENT)
							{
								tx_buff[5] = ((u16)((main_current-ZERO_CURRENT) * MV_PER_MA))>>8;
								tx_buff[6] = (u16)((main_current-ZERO_CURRENT) * MV_PER_MA);
							}
							else
							{
								tx_buff[5] = 0;
								tx_buff[6] = 0;
							}
							tx_buff[7] = 0;
							for(i=0;i<(CMD_MAIN_CURRENT_LENGTH+4);i++)
							{
								tx_buff[7] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_MAIN_CURRENT_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;						
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
					else if(payload[0] == BATT_VOLTAGE_CMD)
					{
						tx_buff[2] = CMD_BATT_VOLTAGE_LENGTH;
						if(state_normal == TRUE)
						{	
							tx_buff[4] = BATT_VOLTAGE_CMD;
							if(Voltage > MIN_VOLTAGE)
							{
								tx_buff[5] = (((u16)((Voltage-MIN_VOLTAGE) * MV_PER_MV))+21000)>>8;
								tx_buff[6] = ((u16)((Voltage-MIN_VOLTAGE) * MV_PER_MV))+21000;
							}
							else
							{
								tx_buff[5] = 0x52;
								tx_buff[6] = 0x08;			//0x5208 = 21000mV = MIN_VOLTAGE
							}
							tx_buff[7] = 0;
							for(i=0;i<(CMD_BATT_VOLTAGE_LENGTH+4);i++)
							{
								tx_buff[7] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_BATT_VOLTAGE_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;						
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
#endif
					else if(payload[0] == BATT_TEMP_CMD)
					{
						tx_buff[2] = CMD_BATT_TEMP_LENGTH;					
						if(state_normal == TRUE)
						{	
							tx_buff[4] = BATT_TEMP_CMD;
							tx_buff[5] = BATT_TEMP_CMD;
							tx_buff[6] = 0;
							for(i=0;i<(CMD_BATT_TEMP_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(CMD_BATT_TEMP_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
						else
						{
							tx_buff[2] = ERROR_FLAG_LENGTH;						
							tx_buff[4] = ERROR_FLAG;
							tx_buff[5] = error_flag_num;
							tx_buff[6] = 0;
							for(i=0;i<(ERROR_FLAG_LENGTH+4);i++)
							{
								tx_buff[6] ^= tx_buff[i];
							}
							for(i=0;i<(ERROR_FLAG_LENGTH+5);i++)
							{
								usart1_send(tx_buff[i]);
							}
						}
					}
					else if(payload[0] == RST_CMD)
					{
						tx_buff[2] = CMD_RST_LENGTH;										
						tx_buff[4] = RST_CMD;
						tx_buff[5] = RST_SUCCESS;
						tx_buff[6] = 0;
						state_normal = TRUE;
//						encoder_left_sigma = 0;
//						encoder_right_sigma = 0;
						for(i=0;i<(CMD_RST_LENGTH+4);i++)
						{
							tx_buff[6] ^= tx_buff[i];
						}
						rst_state = TRUE;
						for(i=0;i<(CMD_RST_LENGTH+5);i++)
						{
							usart1_send(tx_buff[i]);
						}
					}
					else if(payload[0] == CLR_CMD)
					{
						tx_buff[2] = CMD_CLR_LENGTH;										
						tx_buff[4] = CLR_CMD;
						tx_buff[5] = CLR_SUCCESS;
						tx_buff[6] = 0;
						encoder_left_sigma = 0;
						encoder_right_sigma = 0;
						for(i=0;i<(CMD_RST_LENGTH+4);i++)
						{
							tx_buff[6] ^= tx_buff[i];
						}
						for(i=0;i<(CMD_CLR_LENGTH+5);i++)
						{
							usart1_send(tx_buff[i]);
						}
					}
					syn_state =FALSE;
				}	
			}
			if(state_normal == FALSE)
			{
				pwm_f_l = 0;
				pwm_f_r = 0;
				pwm_e_l = 0;
				pwm_e_r = 0;
			}		
		}
		else
		{
			psx_write_read(psx_buf);
			if((psx_buf[1] == PS2_LED_RED) && (psx_buf[2] == 0x5A) && ((PS_KEY_L1&psx_buf[4]) == 0) && ((PS_KEY_R1&psx_buf[4]) == 0))
			{
/*
				usart1_send(0x55);
				usart1_send(psx_buf[0]);
				usart1_send(psx_buf[1]);
				usart1_send(psx_buf[2]);
				usart1_send(psx_buf[3]);
				usart1_send(psx_buf[4]);
				usart1_send(psx_buf_bak[3]);	
				usart1_send(psx_buf_bak[4]);		
				usart1_send(psx_buf[5]);
				usart1_send(psx_buf[6]);
				usart1_send(psx_buf[7]);
				usart1_send(psx_buf[8]);
*/				
				if(((psx_buf[3] ^ psx_buf_bak[3]) != 0) || ((psx_buf[4] ^ psx_buf_bak[4]) != 0))
				{
					if(((psx_buf_bak[4] & PS_KEY_TURN_DEC) == 0) && ((psx_buf[4] & PS_KEY_TURN_DEC) != 0))		//key turn dec pressed and released
					{
						turn_percent -= 25;
						if(turn_percent <= 25)
							turn_percent = 25;
//						usart1_send(turn_percent);
					}
					if(((psx_buf_bak[4] & PS_KEY_TURN_INC) == 0) && ((psx_buf[4] & PS_KEY_TURN_INC) != 0))		//key turn inc pressed and released
					{
						turn_percent += 25;
						if(turn_percent >= 100)
							turn_percent = 100;
//						usart1_send(turn_percent);
					}

					if(((psx_buf_bak[4] & PS_KEY_SPEED_DEC) == 0) && ((psx_buf[4] & PS_KEY_SPEED_DEC) != 0))		//key speed dec pressed and released
					{
						speed_percent -= 25;
						if(speed_percent <= 25)
							speed_percent = 25;
//						usart1_send(speed_percent);
					}
					if(((psx_buf_bak[4] & PS_KEY_SPEED_INC) == 0) && ((psx_buf[4] & PS_KEY_SPEED_INC) != 0))		//key speed inc pressed and released
					{
						speed_percent += 25;
						if(speed_percent >= 100)
							speed_percent = 100;
//						usart1_send(speed_percent);
					}
					if(((psx_buf_bak[3] & PS_KEY_START) == 0) && ((psx_buf[3] & PS_KEY_START) != 0))		//key turn dec pressed and released
					{
						turn_percent = 100;
						speed_percent = 100;
//						usart1_send(turn_percent);
//						usart1_send(speed_percent);
					}

					if(((psx_buf_bak[3] & PS_KEY_SELECT) == 0) && ((psx_buf[3] & PS_KEY_SELECT) != 0))		//key turn dec pressed and released
					{
						turn_percent = 25;
						speed_percent = 25;
//						usart1_send(turn_percent);
//						usart1_send(speed_percent);
					}
				}
				
				if(((psx_buf[3] & PS_KEY_UP) == 0) || ((psx_buf[3] & PS_KEY_DOWN) == 0) || ((psx_buf[3] & PS_KEY_LEFT) == 0) || ((psx_buf[3] & PS_KEY_RIGHT) == 0))
				{
					pwm_f_l_speed = (short)((speed_percent * MAX_SPEED) * ((((psx_buf[3]^0xff) & PS_KEY_UP) >> 4) - (((psx_buf[3]^0xff) & PS_KEY_DOWN) >> 6))/100);
					pwm_f_r_speed = (short)((speed_percent * MAX_SPEED) * ((((psx_buf[3]^0xff) & PS_KEY_UP) >> 4) - (((psx_buf[3]^0xff) & PS_KEY_DOWN) >> 6))/100); 	
					pwm_f_l_turn = (short)((turn_percent * MAX_SPEED) * ((((psx_buf[3]^0xff) & PS_KEY_LEFT) >> 7) - (((psx_buf[3]^0xff) & PS_KEY_RIGHT) >> 5))/100);
					pwm_f_r_turn = (short)((turn_percent * MAX_SPEED) * ((((psx_buf[3]^0xff) & PS_KEY_LEFT) >> 7) - (((psx_buf[3]^0xff) & PS_KEY_RIGHT) >> 5))/100);
				}
				else
				{
					if(abs(0x7f-psx_buf[8]) > MIN_JOYSTICK)
					{
						pwm_f_l_speed = (short)((speed_percent * MAX_SPEED * ((int)(0x7f-psx_buf[8])))/12800);
						pwm_f_r_speed = (short)((speed_percent * MAX_SPEED * ((int)(0x7f-psx_buf[8])))/12800);	
					}
					else
					{
						pwm_f_l_speed = 0;
						pwm_f_r_speed = 0;
					}
					if(abs(0x80-psx_buf[7]) > MIN_JOYSTICK)
					{
						pwm_f_l_turn = (short)((turn_percent * MAX_SPEED * ((int)(0x80-psx_buf[7])))/12800);
						pwm_f_r_turn = (short)((turn_percent * MAX_SPEED * ((int)(0x80-psx_buf[7])))/12800);
					}
					else
					{
						pwm_f_l_turn = 0;
						pwm_f_r_turn = 0;
					}
				}
				
				if((pwm_f_l_speed-pwm_f_l_turn)>MAX_SPEED)
					pwm_f_l = MAX_SPEED;
				else if((pwm_f_l_speed-pwm_f_l_turn)<-MAX_SPEED)
					pwm_f_l = -MAX_SPEED;
				else
					pwm_f_l = pwm_f_l_speed-pwm_f_l_turn;
				
				if((pwm_f_r_speed+pwm_f_r_turn)>MAX_SPEED)
					pwm_f_r = MAX_SPEED;
				else if((pwm_f_r_speed+pwm_f_r_turn)<-MAX_SPEED)
					pwm_f_r = -MAX_SPEED;
				else
					pwm_f_r = pwm_f_r_speed+pwm_f_r_turn;
				
				if(speed_diff_enable)
				{
					if((pwm_f_l - pwm_f_r) > MAX_DIFF_SPEED)
					{
						pwm_diff_speed = pwm_f_l - pwm_f_r;
						pwm_f_l = pwm_f_l * MAX_DIFF_SPEED / pwm_diff_speed;
						pwm_f_r = pwm_f_r * MAX_DIFF_SPEED / pwm_diff_speed;
					}
					else if((pwm_f_r - pwm_f_l) > MAX_DIFF_SPEED)
					{
						pwm_diff_speed = pwm_f_r - pwm_f_l;
						pwm_f_l = pwm_f_l * MAX_DIFF_SPEED / pwm_diff_speed;
						pwm_f_r = pwm_f_r * MAX_DIFF_SPEED / pwm_diff_speed;
					}
				}
/*
				usart1_send(0x55);
				usart1_send((pwm_f_l_speed&0xff00)>>8);
				usart1_send((pwm_f_l_speed&0x00ff));
				usart1_send((pwm_f_l_turn&0xff00)>>8);
				usart1_send(pwm_f_l_turn&0x00ff);	
*/				
				psx_buf_bak[3] = psx_buf[3];
				psx_buf_bak[4] = psx_buf[4];
				
			}
			else
			{
				pwm_f_l = 0;
				pwm_f_r = 0;
				pwm_e_l = 0;
				pwm_e_r = 0;
			}

/*
			usart1_send(0x55);
			usart1_send((pwm_f_l&0xff00)>>8);
			usart1_send((pwm_f_l&0x00ff)>>0);
			usart1_send((pwm_f_r&0xff00)>>8);
			usart1_send((pwm_f_r&0x00ff)>>0);
*/		
			delay_ms(10);
		}
		if(KEY0 == 0)
		{
			state_normal = FALSE;
			error_flag_num = STOP_KEY_ERROR_CODE;
		}
#ifdef	VOLTAGE_DET	
		if(Voltage < MIN_VOLTAGE)
		{
				state_normal = FALSE;
				error_flag_num = LOW_VOLTAGE_ERROR_CODE;
		}
#endif
#ifdef	CURRENT_DET	
		if(main_current > BUZZ_CURRENT)
		{
			BUZZER_ON;
			counter_buzz_5ms = 0;
		}
/*		else
		{
			if(counter_buzz_5ms > BUZZ_5MS_3S_COUNTER)
			{
				counter_buzz_5ms = BUZZ_5MS_3S_COUNTER + 1;
			}
		}
*/		
		if(main_current > MAX_CURRENT)
		{
			state_normal = FALSE;
			error_flag_num = MAX_CURRENT_ERROR_CODE;
		}
#endif
#ifdef	COMM_DET			
		if(counter_5ms > MAX_5MS_COUNTER)
		{
			state_normal = FALSE;
			error_flag_num = NO_COM_DATA_ERROR_CODE;
		}
#endif
#ifdef	ACC_DET			
		if(counter_accident > MAX_ACC_COUNTER)
		{
			state_normal = FALSE;
			error_flag_num = ACCIDENT_ERROR_CODE;
		}
#endif
		if(DBG_ADC)
		{
			usart2_send(0x55);
			usart2_send(main_current>>8);
			usart2_send(main_current);	
			usart2_send(Voltage>>8);
			usart2_send(Voltage);	
		}


		
			// SPEED BAT REMAIN_DIST BAT_TEMP RST CLR	
			//55 aa 09 01 01 00 10 00 10 00 10 00 10 f6
			//55 aa 09 01 01 00 00 00 00 00 00 00 00 f6
			//55 aa 09 01 01 00 01 02 03 04 05 06 07 f6
			//55 aa 09 01 01 00 10 00 20 00 00 00 00 c6
			//55 aa 09 01 01 ff f0 00 20 00 00 00 00 d9
			//55 aa 02 01 02 00 fe
			//55 aa 02 01 03 00 ff
			//55 aa 02 01 04 00 f8
			//55 aa 02 01 05 00 f9
			//55 aa 02 01 06 00 fa
			//55 aa 02 01 07 00 fb
			//55 aa 02 01 08 00 f4 55 aa 02 01 05 00 f9 55 aa 02 01 07 00 fb
	}		
}	
