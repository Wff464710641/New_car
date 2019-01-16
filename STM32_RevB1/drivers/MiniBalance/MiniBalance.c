#include "MiniBalance.h"
#include "math.h"
#include "led.h"
#include "mpu6050.h"
#include "main.h"
#include "iwdg.h"
//#define PI 3.14159265
//#define KP_PID 2
//#define KI_PID 0		//0 for 2.0 1 for 1.5
//#define KD_PID 5

//#define KP_PID_HS 2
//#define KI_PID_HS 0.02
//#define KD_PID_HS 2
//#define	ERROR_MIN_HS	5 		//high speed min error

#define PID_ENABLE	1


#define KP_PID 2
#define KI_PID 0		//0 for 2.0 1 for 1.5
#define KD_PID 5

#define	ERROR_MIN	1

#define KP_PID_HS 2
#define KI_PID_HS 0
#define KD_PID_HS 2
#define	ERROR_MIN_HS	2 		//high speed min error
#define	MAX_SUM_ERROR 10
#define	MAX_PWM				1900

#define ENCODER_HS	10		//high speed and low speed point
#define DBG					0


/**************************************************************************
作者:流星落泪
日期：20150728
库版本：V3.5
**************************************************************************/

/**************************************************************************
函数功能：5MS定时中断函数 5MS控制周期
入口参数：无
返回  值：无
作    者：流星落泪
**************************************************************************/
int Velocity_Pwm_left,Velocity_Pwm_right;
void TIM1_UP_TIM16_IRQHandler(void)  
{    
	if((led_flash%2) == 0)
	{
		LED1_ON;
	}
	else
	{
		LED1_OFF;
	}
	if(TIM1->SR&0X0001)//5ms定时中断
	{   
		IWDG_ReloadCounter(); //喂狗
		counter_buzz_5ms++;
		counter_5ms++;
		led_flash++;
		if(counter_buzz_5ms > BUZZ_5MS_3S_COUNTER)
		{
			counter_buzz_5ms = BUZZ_5MS_3S_COUNTER+1;
			BUZZER_OFF;
		}	
		
		if(counter_5ms > MAX_5MS_COUNTER)
		{
			counter_5ms = MAX_5MS_COUNTER+1;
		}	
		TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 
//		if(encoder_rst_flag == 1)
//		{
//			encoder_left_sigma=0;
//			encoder_right_sigma=0;
//		}
		readEncoder();                                           //===读取编码器的值
		if(((Encoder_Left<LOW_SPEED_ENCODER) && (pwm_f_l > LOW_SPEED_PRESET)) || 
				((Encoder_Right<LOW_SPEED_ENCODER) && (pwm_f_r > LOW_SPEED_PRESET)))
		{
			counter_accident++;
		}
		else
		{
			counter_accident = 0;
		}
		encoder_left_sigma += Encoder_Left;
		encoder_right_sigma += Encoder_Right;
 		tx_encoder_buff[5] = (encoder_left_sigma & 0xff00)>>8;
 		tx_encoder_buff[6] = (encoder_left_sigma & 0x00ff);
 		tx_encoder_buff[7] = (encoder_right_sigma & 0xff00)>>8;
 		tx_encoder_buff[8] = (encoder_right_sigma & 0x00ff);


		Get_battery_volt();   //===获取ADC各个通道采集的值	          
													//读取电机电流，电池电压，转向角度
		Velocity_Pwm_left = velocity_left(Encoder_Left);
		Velocity_Pwm_right = velocity_right(Encoder_Right);       //===速度环PID控制
		if(PID_ENABLE)
		{
			Moto1=Velocity_Pwm_left;                 //===计算左轮电机最终PWM
			Moto2=Velocity_Pwm_right;                 //===计算右轮电机最终PWM
		}
		else
		{
			Moto1 = pwm_f_l*1900/80;
			Moto2 = pwm_f_r*1900/80;
		}
		Xianfu_Pwm();                                            //===PWM限幅
		if(Turn_Off(Angle_Balance,Voltage)==0)                   //===如果不存在异常
		{
			Set_Pwm(Moto1,Moto2);			//===赋值给PWM寄存器  
		}            
	} 
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改Movement的值，比如，改成-600和600就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：流星落泪
**************************************************************************/
int velocity_left(int encoder_tmp)
{  
	static int Velocity_t0,Velocity_t1;		//t0 当前时刻，t1前一时刻，t2前前一时刻
	static int encoder_left_pro_set=5;
	static int error_t0=0,error_t1=0;
	static int sum_error=0;
	encoder_left_pro_set = pwm_f_l;
//		encoder_left_pro_set = 0x100;
	if(DBG)
	{
		usart2_send(0x55);
		usart2_send(encoder_left_pro_set);
		usart2_send(encoder_tmp);
	}
	error_t0 = encoder_left_pro_set-encoder_tmp;
	sum_error += error_t0;
	if(sum_error>MAX_SUM_ERROR)	sum_error = MAX_SUM_ERROR;
	if(sum_error<-MAX_SUM_ERROR)	sum_error = -MAX_SUM_ERROR;

	if(encoder_left_pro_set > ENCODER_HS)
	{
		if(abs(error_t0) < ERROR_MIN_HS)
		{
			Velocity_t0 = Velocity_t1;
		}
		else
		{
			Velocity_t0 = Velocity_t1 + error_t0*KP_PID_HS + sum_error*KI_PID_HS + (error_t0-error_t1)*KD_PID_HS;
		}
	}
	else
	{
		if(abs(error_t0) < ERROR_MIN)
		{
			Velocity_t0 = Velocity_t1;
		}
		else
		{	
			Velocity_t0 = Velocity_t1 + error_t0*KP_PID + sum_error*KI_PID + (error_t0-error_t1)*KD_PID;
		}
	}

	if(DBG)
	{
		usart2_send(error_t0);
	}
	if(Velocity_t0<-MAX_PWM) Velocity_t0=-MAX_PWM;	
	if(Velocity_t0>MAX_PWM) Velocity_t0=MAX_PWM;	

	Velocity_t1 = Velocity_t0;
	error_t1 = error_t0;
	return Velocity_t0;
}

int velocity_right(int encoder_tmp)
{  
	static int Velocity_t0,Velocity_t1;		//t0 当前时刻，t1前一时刻，t2前前一时刻
	static int encoder_right_pro_set=5;
	static int error_t0=0,error_t1=0;
	static int sum_error=0;
	encoder_right_pro_set = pwm_f_r;
	if(DBG)
	{
		usart2_send(0xaa);
		usart2_send(encoder_right_pro_set);
		usart2_send(encoder_tmp);
	}
	error_t0 = encoder_right_pro_set-encoder_tmp;
	sum_error += error_t0;
	if(sum_error>MAX_SUM_ERROR)	sum_error = MAX_SUM_ERROR;
	if(sum_error<-MAX_SUM_ERROR)	sum_error = -MAX_SUM_ERROR;

	if(encoder_right_pro_set > ENCODER_HS)
	{
		if(abs(error_t0) < ERROR_MIN_HS)
		{
			Velocity_t0 = Velocity_t1;
		}
		else
		{
			Velocity_t0 = Velocity_t1 + error_t0*KP_PID_HS + sum_error*KI_PID_HS + (error_t0-error_t1)*KD_PID_HS;
		}
	}
	else
	{
		if(abs(error_t0) < ERROR_MIN)
		{
			Velocity_t0 = Velocity_t1;
		}
		else
		{	
			Velocity_t0 = Velocity_t1 + error_t0*KP_PID + sum_error*KI_PID + (error_t0-error_t1)*KD_PID;
		}
	}
	if(DBG)
	{
		usart2_send(error_t0);
	}
	if(Velocity_t0<-MAX_PWM) Velocity_t0=-MAX_PWM;	
	if(Velocity_t0>MAX_PWM) Velocity_t0=MAX_PWM;	
	Velocity_t1 = Velocity_t0;
	error_t1 = error_t0;
	return Velocity_t0;
}


/**************************************************************************
函数功能：赋值给PWM寄存器    控制电机反正转  并且赋给寄存器pwm值
入口参数：左轮PWM、右轮PWM
返回  值：无
作    者：流星落泪
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	
	if(moto1<0)
	{
		AIN2=1;
		AIN1=0;
	}
	else
	{
		AIN2=0;
		AIN1=1;
	}
	PWMA=myabs(moto1);
	if(moto2<0)
	{
		BIN1=0;
		BIN2=1;
	}
	else
	{
		BIN1=1;
		BIN2=0;
	}
	PWMB=myabs(moto2);
}

/**************************************************************************
函数功能：读取编码器的数据并进行数据类型转换
入口参数：无
返回  值：无
作    者：流星落泪
**************************************************************************/
void readEncoder(void)
{
	u16 Encoder_L,Encoder_R;       //===左右编码器的脉冲计数
	Encoder_R = TIM4 -> CNT;       //===获取正交解码1数据	
	TIM4 -> CNT=0;                 //===计数器清零  
	Encoder_L= TIM2 -> CNT;        //===获取正交解码2数据	
	TIM2 -> CNT=0;	               //===计数器清零

	if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  
	//=这个处理的原因是：编码器到0后会跳到65000向下计数，这样处理方便我们在控制程序中使用
	if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;
	Encoder_Left=-Encoder_Left;//这里取反是因为，平衡小车的两个电机是旋转了180度安装的，为了保证前进后退时候的编码器数据符号一致
}

/**************************************************************************
函数功能：限制PWM赋值   限制平衡车的速度
入口参数：无
返回  值：无
作    者：流星落泪
**************************************************************************/
void Xianfu_Pwm(void)
{	
//	int Amplitude=1950;    //===PWM满幅是3600 限制在3500 20khz
	if(Moto1<-MAX_PWM) 
		Moto1=-MAX_PWM;	
	if(Moto1>MAX_PWM)  
		Moto1=MAX_PWM;	
	if(Moto2<-MAX_PWM) 
		Moto2=-MAX_PWM;	
	if(Moto2>MAX_PWM)  
		Moto2=MAX_PWM;		
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
作    者：流星落泪
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;  //临时变量 返回值
	if(KEY0==0)				//if key0 is press on, stop moto, current, temperature, unexpected stop
	{
		temp=1;                                            
		PWMA=0;
		PWMB=0;			//关闭PWM输出  这样有刹车   如果关闭反正转则没有刹车
								//===可自行增加主板温度过高时关闭电机
	}
	else
	{
		temp=0;	
	}
	return temp;			
}

/**********************************************************
防止瞬间启动函数
作者：流星落泪
时间：20150714
***********************************************************/

void Qidongshigu(void)    //启动防止突然找平衡点函数
{
	static u8 i=0; //临时变量

	if(temp9==1)   //踏板开关按下开始做动作
	  {
         if(temp10>0)   //以下是启动时间防止瞬间找平衡点的算法 
			{
				if(++i>=10)     //50ms改变一次启动角度值
				{
					i=0;
					//角度值为正的时间减
					if(--temp10==0)
					{
						temp8=1;  //当踏板水平时 temp8赋值为1
					}
				}
			}
			else
			{
				if(++i>=10)       //50ms改变一次启动角度值
				{
					i=0;
					 //角度值为负时加
					if(++temp10==0)
					{
						temp8=1;  //当踏板水平时 temp8赋值为1
					}
				}
			}	
	    }
	else
	{
		
		//temp10=angle;  //当使用卡尔曼算法打开
		//踏板开关没有按下时赋加速度的实际值
		temp10=Pitch;    //当使用四元算法打开         
		//防止启动突然找平衡点变量赋值   这里赋给的是加速度测得的角度
		temp6=temp7;  //转向零点检测
		temp8=0;      //踏板开关没有按下 temp8赋值为0
	}
	
}

void yaokongkaiqi(void)   //遥控启动报警函数
{
	static u8 i=0;
	
	if(yaokongqidongbaojing==1) LED1=1;
	if(++i>200) i=1,yaokongqidongbaojing=0, LED1=0;
}

