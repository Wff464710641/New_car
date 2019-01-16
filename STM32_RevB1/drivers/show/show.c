#include "show.h"
#include "MiniBalance.h"
/**************************************************************************
���ߣ���������
��汾��v3.5
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
��    �ߣ���������
**************************************************************************/
static u32 Count;
void oled_show(void)
{
	  Count=0;
		OLED_Display_On();  //��ʾ����
		//=============��ʾ�˲���=======================//	
		                        OLED_ShowString(00,0,"WAY-");
		                        OLED_ShowNumber(30,0, Way_Angle,1,12);
	         if(Way_Angle==1)	OLED_ShowString(45,0,"DMP");
		else if(Way_Angle==2)	OLED_ShowString(45,0,"Kalman");
		else if(Way_Angle==3)	OLED_ShowString(45,0,"Hubu");
		//=============��ʾ�¶�=======================//	
		                      OLED_ShowString(00,10,"Wendu");
		                      OLED_ShowNumber(45,10,Temperature/10,2,12);
		                      OLED_ShowNumber(68,10,Temperature%10,1,12);
		                      OLED_ShowString(58,10,".");
		                      OLED_ShowString(80,10,"`C");
		//=============��ʾ������1=======================//	
		                        OLED_ShowString(00,20,"Enco1");
		if( Encoder_Left<0)		OLED_ShowString(45,20,"-"),
		                        OLED_ShowNumber(65,20,-Encoder_Left,5,12);
		else                 	OLED_ShowString(45,20,"+"),
		                        OLED_ShowNumber(65,20, Encoder_Left,5,12);
  	//=============��ʾ������2=======================//		
		                        OLED_ShowString(00,30,"Enco2");
		if(Encoder_Right<0)		OLED_ShowString(45,30,"-"),
		                        OLED_ShowNumber(65,30,-Encoder_Right,5,12);
		else               		OLED_ShowString(45,30,"+"),
		                        OLED_ShowNumber(65,30,Encoder_Right,5,12);	
		//=============��ʾ��ѹ=======================//
								  OLED_ShowString(00,40,"Volta");
								  OLED_ShowString(58,40,".");
								  OLED_ShowString(80,40,"V");
								  OLED_ShowNumber(45,40,Voltage/100,2,12);
								  OLED_ShowNumber(68,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	  OLED_ShowNumber(62,40,0,2,12);
		//=============��ʾ�Ƕ�=======================//
		                            OLED_ShowString(0,50,"Angle");
		if(Angle_Balance<0)		    OLED_ShowNumber(45,50,angle+360,3,12);
		
		else					    OLED_ShowNumber(45,50,angle,3,12);
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
	}
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ���������
**************************************************************************/
void DataScope(void)
{   
	  if(++Count==1)
		{	
			OLED_Clear();  
			OLED_Display_Off();		
		}	
			DataScope_Get_Channel_Data( Angle_Balance, 1 );
			DataScope_Get_Channel_Data(YY3, 2 );
			DataScope_Get_Channel_Data( Accel_X/100.00, 3 ); 
			DataScope_Get_Channel_Data( Accel_Z/100-165.00,4 );   
			DataScope_Get_Channel_Data(Gyro_Y, 5 );        //����Ҫ��ʾ�������滻0������
			DataScope_Get_Channel_Data(Gyro_Z/100.00 , 6 );//����Ҫ��ʾ�������滻0������
			DataScope_Get_Channel_Data(temp2, 7 );
			DataScope_Get_Channel_Data(Encoder_Right, 8 ); 
			DataScope_Get_Channel_Data(Encoder_Left, 9 );  
			DataScope_Get_Channel_Data( Voltage/100.00 , 10);
			Send_Count = DataScope_Data_Generate(10);
		for( i = 0 ; i < Send_Count; i++) 
		{
			while((USART1->SR&0X40)==0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		delay_ms(50); //20HZ
}
