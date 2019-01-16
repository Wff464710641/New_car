#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "show.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "timer.h"
#include "mpu6050.h"
#include "usart2.h"
#include "adc.h"
#include "DataScope_DP.h"

#define LED1_ON		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
#define LED1_OFF		GPIO_SetBits(GPIOC,GPIO_Pin_2);

#define BUZZER_OFF		GPIO_ResetBits(GPIOE,GPIO_Pin_3);
#define BUZZER_ON			GPIO_SetBits(GPIOE,GPIO_Pin_3);



#define PS2_CLK_1		GPIO_SetBits(GPIOC,GPIO_Pin_8);
#define PS2_CLK_0		GPIO_ResetBits(GPIOC,GPIO_Pin_8);

#define PS2_ATT_1		GPIO_SetBits(GPIOC,GPIO_Pin_9);
#define PS2_ATT_0		GPIO_ResetBits(GPIOC,GPIO_Pin_9);

#define PS2_CMD_1		GPIO_SetBits(GPIOA,GPIO_Pin_12);
#define PS2_CMD_0		GPIO_ResetBits(GPIOA,GPIO_Pin_12);

#define PS2_DAT		  GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)

#define PS2_LED_RED  0x73				//ps2 type led red mod


#define START_CMD				0x01		//ps2 start cmd
#define ASK_DAT_CMD			0x42		//ps2 ask date cmd
#define PS2_MAX_BUF_SIZE	9

#define PS_KEY_TURN_DEC					0x80
#define PS_KEY_TURN_INC					0x20
#define PS_KEY_SPEED_DEC				0x40
#define PS_KEY_SPEED_INC				0x10



#define PS_KEY_SELECT						0x01
#define PS_KEY_START						0x08
#define PS_KEY_UP								0x10
#define PS_KEY_DOWN							0x40
#define PS_KEY_LEFT							0x80
#define PS_KEY_RIGHT						0x20

#define PS_KEY_L1								0x04
#define PS_KEY_R1								0x08


#define TIMER_100MS

#define FALSE										0
#define TRUE										1

#define CURRENT_DET
#define COMM_DET
#define ACC_DET



#define STATE_IDLE							0
#define STATE_RX_HDR1						1
#define STATE_RX_HDR2						2
#define STATE_RX_LENGTH					3
#define STATE_RX_SEQ						4
#define STATE_RX_PAYLOAD				5
#define STATE_RX_PARITY					6

#define SPEED_CMD					1
#define BATT_CMD					2			//battery capacity
#define REMAIN_DIST_CMD		3			//battery can support remain distance
#define BATT_TEMP_CMD			4			//battery temperature
#define RST_CMD						5
#define CLR_CMD						6
#define MAIN_CURRENT_CMD	7			//main current
#define BATT_VOLTAGE_CMD	8			//battery voltage
#define FIRMWARE_READ_CMD	9			
#define HARDWARE_READ_CMD	10			

#define RST_SUCCESS				1
#define RST_FAIL					0

#define CLR_SUCCESS				1
#define CLR_FAIL					0



#define	FRAME_HEADER1			0x55
#define	FRAME_HEADER2			0xaa


#define	TX_FRAME_HDR1			0x55
#define	TX_FRAME_HDR2			0xaa


#define CMD_SPEED_LENGTH					9
#define CMD_BATT_LENGTH						2
#define CMD_REMAIN_DIST_LENGTH		2		
#define CMD_MAIN_CURRENT_LENGTH		3		
#define CMD_BATT_VOLTAGE_LENGTH		3
#define CMD_BATT_TEMP_LENGTH			2		
#define CMD_RST_LENGTH						2
#define CMD_CLR_LENGTH						2
#define	CMD_READ_FIRMWARE_LENGTH	2
#define	CMD_READ_HARDWARE_LENGTH	2
#define ERROR_FLAG_LENGTH					2	

#define MV_PER_MA									6.5
#define MV_PER_MV									8.86
#define ZERO_CURRENT							0x0610
#undef	PM1
#undef	TSINGHUA

#ifdef	PM1
	#define	MOTO_SPEED								120
	#define	ENCODER_PULSE							1024
#else
	#ifdef TSINGHUA
		#define	MOTO_SPEED								120				//75 RPM
	#else	
		#define	MOTO_SPEED								75				//75 RPM
	#endif
	#define	ENCODER_PULSE							1600
#endif
#define	MOTO_GEAR									20				
#define	WHEEL_GEAR								25
#define	ENCODER_GEAR							25
#define	PID_FREQ									50				//20ms  50Hz

#define	MAX_STEP									MAX_SPEED/2				// 80 is max, means no step
//#define	MAX_SPEED									60			//5ms
//#define	MAX_SPEED									80				//20ms		75rpm/m	-- 25:20齿轮比例 75*20/25=60 60/60=1hz 1*25/10（齿轮比）=2.5 * 1600线 = 4000线 4000/(1000ms/20ms) = 80
#define	MAX_SPEED									1.0*MOTO_SPEED*MOTO_GEAR*ENCODER_PULSE/WHEEL_GEAR/60/PID_FREQ

#define	MAX_DIFF_SPEED			MAX_SPEED/2


#define MAX_PAYLOAD_NUM			16
#define RX_BUFF_MAX					32
#define MAX_TX_PAYLOAD_NUM	16 

#define ERROR_FLAG					0xff  

#define	MIN_VOLTAGE					2369		//7 cell min voltage 21V 21/110*10/3.3*4096
//#define	MIN_VOLTAGE					3200			//10 cell min voltage 33V 33/108.45*8.45/3.3*4096 = 3191
//#define	MAX_VOLTAGE					4060			//10 cell min voltage 42V 42/108.45*8.45/3.3*4096 = 4061
#define	MAX_VOLTAGE					3317			//7 cell max voltage 29.4V 29.4/110*10/3.3*4096 = 4061
#define	MAX_DIST						50				//MAX distance 
//#define	MAX_CURRENT					25			//0.005ohm/4 = 0.00125ohm *20A(max current) = 0.025V
#define	MAX_CURRENT					1998			//0A = 2.5V 40mV/A 18A=720mV 2.5V + 0.72V = 3.22V 12bit adc max = 4096 adc = 3.22/3.3*4096 = 3996 10K 10K /2 = 1998
#define	BUZZ_CURRENT				1924			//0A = 2.5V 40mV/A 15A=600mV 2.5V + 0.6V = 3.1V 12bit adc max = 4096 adc = 3.1/3.3*4096 = 3847 10K 10K /2 = 1924
//#define	BUZZ_CURRENT				1675			//0A = 2.5V 40mV/A 2.5A=100mV 2.5V + 0.1V = 2.6V 12bit adc max = 4096 adc = 2.6/3.3*4096 = 3227 10K 10K /2 = 1614
#define	MIN_JOYSTICK				32


#ifdef	TIMER_5MS																			//0.025/3.3*4096 = 31.03
#define MAX_5MS_COUNTER			200
#elif defined TIMER_100MS
#define MAX_5MS_COUNTER			50
#endif

#define BUZZ_5MS_3S_COUNTER			160

//1s	200 times 1:17 1024/cycle	1024*17/200 = 87.04 43*3.14 =135cm

#define	STOP_KEY_ERROR_CODE				16
#define	LOW_VOLTAGE_ERROR_CODE		1
#define	MAX_CURRENT_ERROR_CODE		2
#define	NO_COM_DATA_ERROR_CODE		3
#define	ACCIDENT_ERROR_CODE				4

#define	LOW_SPEED_ENCODER			3 
#define	LOW_SPEED_PRESET			10 
#define	MAX_ACC_COUNTER				4

extern s8  temp10;  //启动瞬间角度值变量
extern u8  temp9;   //踏板开关变量
extern u8  temp8;
extern s16 temp7;   //转向电位器信号采集变量
extern s16 temp6;   //电位器安装偏置 防止车跑偏变量
extern s16 temp76;

extern u8		temp5;   //按键消抖延时变量
extern u8		temp4;   //蓝牙变量
extern u16	main_current;   //过流保护变量
extern int	temp2;   //超速保护变量
extern u8		temp1;   //启动变量
extern u8		temp0;   //低中高三挡模式
extern u16 xiansu;   //限速变量
//extern u8  xiansu1;    //这两个是限速变量
extern short	pwm_f_l,pwm_f_r;	//front left speed front right speed
extern short	pwm_e_l,pwm_e_r;	//end left speed end right speed
extern u8 error_flag_num;	//state error num
extern u8 state_normal;		//car state:normal non-normal if non-nomarl stop car
extern u8 rcv_whole_packet;
extern u8 rst_state;
extern u8 counter_5ms;
extern u16 counter_buzz_5ms;
extern u8 led_flash;
extern u8 counter_accident;
extern u8 rx_buff[RX_BUFF_MAX];
extern u8 ptr_start,ptr_end;

extern u8 machine_state;
extern u8 rx_buf_tmp,rx_parity,syn_state,frame_seq;
extern u8 cmd_length;
extern u8 payload_num;
extern u8 payload[MAX_PAYLOAD_NUM];



extern long encoder_left_sigma,encoder_right_sigma;		//left & right encoder sigma
extern u8 encoder_rst_flag;
extern u8 tx_encoder_buff[MAX_TX_PAYLOAD_NUM];

extern  u8 QIDONG;        //蓝牙启动变量
extern  u8 CHEDENG;       //蓝牙车灯控制变量
extern  u8 chednegmoshi;   //车灯模式控制变量
extern  u8 MOSHI;         //蓝牙模式控制变量
extern  u8 YINYE;         //蓝牙音乐控制变量
extern  u8 tabanyy1;     //上车报警变量
extern u8 yaokongqidongbaojing; //遥控启动报警变量

extern float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;   //陀螺仪 加速度变量  

extern u8  Way_Angle;                             	//获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern int Encoder_Left,Encoder_Right;             	//左右编码器的脉冲计数
extern int Moto1,Moto2;                            	//电机PWM变量 应是motor的 向moto致敬	
extern u8  Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量
extern u8  Flag_Stop,Flag_Show;                 		//停止标志位和 显示标志位 默认停止 显示打开
extern u16 Voltage,Voltage_Zheng,Voltage_Xiao;			//电池电压采样相关的变量
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
extern float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
extern int Temperature;
/********************************************************************************************/
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
/********************************************************************************************/
//#include "STM32_I2C.h"
#include "IOI2C.h"
#include "usart1.h"
#include "delay.h"
#include "DMP\inv_mpu.h"
#include "DMP\inv_mpu_dmp_motion_driver.h"
#include "DMP\dmpKey.h"
#include "DMP\dmpmap.h"

#endif

