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
����:��������
���ڣ�20150728
��汾��V3.5
**************************************************************************/

/**************************************************************************
�������ܣ�5MS��ʱ�жϺ��� 5MS��������
��ڲ�������
����  ֵ����
��    �ߣ���������
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
	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
	{   
		IWDG_ReloadCounter(); //ι��
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
		TIM1->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
//		if(encoder_rst_flag == 1)
//		{
//			encoder_left_sigma=0;
//			encoder_right_sigma=0;
//		}
		readEncoder();                                           //===��ȡ��������ֵ
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


		Get_battery_volt();   //===��ȡADC����ͨ���ɼ���ֵ	          
													//��ȡ�����������ص�ѹ��ת��Ƕ�
		Velocity_Pwm_left = velocity_left(Encoder_Left);
		Velocity_Pwm_right = velocity_right(Encoder_Right);       //===�ٶȻ�PID����
		if(PID_ENABLE)
		{
			Moto1=Velocity_Pwm_left;                 //===�������ֵ������PWM
			Moto2=Velocity_Pwm_right;                 //===�������ֵ������PWM
		}
		else
		{
			Moto1 = pwm_f_l*1900/80;
			Moto2 = pwm_f_r*1900/80;
		}
		Xianfu_Pwm();                                            //===PWM�޷�
		if(Turn_Off(Angle_Balance,Voltage)==0)                   //===����������쳣
		{
			Set_Pwm(Moto1,Moto2);			//===��ֵ��PWM�Ĵ���  
		}            
	} 
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ����޸�Movement��ֵ�����磬�ĳ�-600��600�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ���������
**************************************************************************/
int velocity_left(int encoder_tmp)
{  
	static int Velocity_t0,Velocity_t1;		//t0 ��ǰʱ�̣�t1ǰһʱ�̣�t2ǰǰһʱ��
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
	static int Velocity_t0,Velocity_t1;		//t0 ��ǰʱ�̣�t1ǰһʱ�̣�t2ǰǰһʱ��
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
�������ܣ���ֵ��PWM�Ĵ���    ���Ƶ������ת  ���Ҹ����Ĵ���pwmֵ
��ڲ���������PWM������PWM
����  ֵ����
��    �ߣ���������
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
�������ܣ���ȡ�����������ݲ�������������ת��
��ڲ�������
����  ֵ����
��    �ߣ���������
**************************************************************************/
void readEncoder(void)
{
	u16 Encoder_L,Encoder_R;       //===���ұ��������������
	Encoder_R = TIM4 -> CNT;       //===��ȡ��������1����	
	TIM4 -> CNT=0;                 //===����������  
	Encoder_L= TIM2 -> CNT;        //===��ȡ��������2����	
	TIM2 -> CNT=0;	               //===����������

	if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  
	//=��������ԭ���ǣ���������0�������65000���¼��������������������ڿ��Ƴ�����ʹ��
	if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;
	Encoder_Left=-Encoder_Left;//����ȡ������Ϊ��ƽ��С���������������ת��180�Ȱ�װ�ģ�Ϊ�˱�֤ǰ������ʱ��ı��������ݷ���һ��
}

/**************************************************************************
�������ܣ�����PWM��ֵ   ����ƽ�⳵���ٶ�
��ڲ�������
����  ֵ����
��    �ߣ���������
**************************************************************************/
void Xianfu_Pwm(void)
{	
//	int Amplitude=1950;    //===PWM������3600 ������3500 20khz
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
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
��    �ߣ���������
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;  //��ʱ���� ����ֵ
	if(KEY0==0)				//if key0 is press on, stop moto, current, temperature, unexpected stop
	{
		temp=1;                                            
		PWMA=0;
		PWMB=0;			//�ر�PWM���  ������ɲ��   ����رշ���ת��û��ɲ��
								//===���������������¶ȹ���ʱ�رյ��
	}
	else
	{
		temp=0;	
	}
	return temp;			
}

/**********************************************************
��ֹ˲����������
���ߣ���������
ʱ�䣺20150714
***********************************************************/

void Qidongshigu(void)    //������ֹͻȻ��ƽ��㺯��
{
	static u8 i=0; //��ʱ����

	if(temp9==1)   //̤�忪�ذ��¿�ʼ������
	  {
         if(temp10>0)   //����������ʱ���ֹ˲����ƽ�����㷨 
			{
				if(++i>=10)     //50ms�ı�һ�������Ƕ�ֵ
				{
					i=0;
					//�Ƕ�ֵΪ����ʱ���
					if(--temp10==0)
					{
						temp8=1;  //��̤��ˮƽʱ temp8��ֵΪ1
					}
				}
			}
			else
			{
				if(++i>=10)       //50ms�ı�һ�������Ƕ�ֵ
				{
					i=0;
					 //�Ƕ�ֵΪ��ʱ��
					if(++temp10==0)
					{
						temp8=1;  //��̤��ˮƽʱ temp8��ֵΪ1
					}
				}
			}	
	    }
	else
	{
		
		//temp10=angle;  //��ʹ�ÿ������㷨��
		//̤�忪��û�а���ʱ�����ٶȵ�ʵ��ֵ
		temp10=Pitch;    //��ʹ����Ԫ�㷨��         
		//��ֹ����ͻȻ��ƽ��������ֵ   ���︳�����Ǽ��ٶȲ�õĽǶ�
		temp6=temp7;  //ת�������
		temp8=0;      //̤�忪��û�а��� temp8��ֵΪ0
	}
	
}

void yaokongkaiqi(void)   //ң��������������
{
	static u8 i=0;
	
	if(yaokongqidongbaojing==1) LED1=1;
	if(++i>200) i=1,yaokongqidongbaojing=0, LED1=0;
}

