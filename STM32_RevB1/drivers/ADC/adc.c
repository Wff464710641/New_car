#include "main.h"
#include "adc.h"

/**************************************************************************
���ߣ���������
��汾��v3.5
**************************************************************************/

/**************************************************************************
�������ܣ�ACD��ʼ��
��ڲ�������
����  ֵ����
��    �ߣ���������
**************************************************************************/
void  Adc_Init(void)
{    

	ADC_InitTypeDef ADC_InitStructure; //adc�ṹ�嶨��
	GPIO_InitTypeDef GPIO_InitStructure;//gpio�ṹ�嶨��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA,ENABLE);//ʹ�� gpioa adc1 ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����adc�ķ�Ƶ����  adc�����14m
	
 	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//��������ģʽ  ģ������
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼPA4--PA7
	
	ADC_DeInit(ADC1);   //������ADC1ȫ���Ĵ�������ȱʡ   ��λADC1
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;//����ADC����ͨ��
	ADC_InitStructure.ADC_ScanConvMode=DISABLE; //��ͨ��ģʽ  ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;//����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//�����������
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//���������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel=8;//ͨ����
	ADC_Init(ADC1,&ADC_InitStructure);//��ʼADC1
	
	ADC_Cmd(ADC1,ENABLE);//ʹ��ADC1
	ADC_ResetCalibration(ADC1);  //����adc1У׼
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�У׼����
	ADC_StartCalibration(ADC1); //����ADC1У׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�У׼����
	delay_ms(1);
	Get_battery_volt();   
}		

/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����  ֵ��ADת�����
��    �ߣ���������
**************************************************************************/
u16 Get_Adc(u8 ch,u8 times )   
{
// 	//����ת������	  		 
// 	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
// 	ADC1->SQR3|=ch;		  			    
// 	ADC1->CR2|=1<<22;       //��������ת��ͨ�� 
// 	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
// 	return ADC1->DR;		//����adcֵ	
	
	u32 temp_val=0;//50��ת������ĺ�
	u8 t;//��ʱ����
	ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);    //ͨ������
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ʹ�����ת��
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//�ȴ�ת������
	
	for(t=0;t<times;t++)  //ѭ���ۼ�50��
	{
		temp_val+=ADC_GetConversionValue(ADC1); //��ȡADC�ϴ�ת����ֵ ����50���ۼ�
	}
	return temp_val/times;//�����50�ε�ƽ��ֵ
}

/**************************************************************************
�������ܣ���ȡ��ص�ѹ
��ڲ�������
����  ֵ����
���ߣ���������
��汾��V3.5
**************************************************************************/
void Get_battery_volt(void)   
{  
//����ADC�ɼ�ͨ�����ĸ����Ǹ��Ϳ���
		Voltage = Get_Adc(ADC_Channel_4 ,50);           			//PA4 	�����ɼ�  1638��40A
																													//PA4		��ѹ�ɼ�	24V/11 = 2.1818 / 3.3 * 4096 =  0x0a94
		main_current = Get_Adc(ADC_Channel_5 ,5);         		//PA5		0A = 2.5V  0x060f 40mV/A 15A=600mV 2.5V + 0.6V = 3.1V/2(10K + 10K) adc = 0x0783

		
//		temp7 = (Get_Adc(ADC_Channel_6 ,50)-2048)/2.7;   	//PA6  ת��ת���λ����ѹ��AD�ɼ�
//		temp7 = Get_Adc(ADC_Channel_6 ,50);   							//PA6  ת��ת���λ����ѹ��AD�ɼ�
// 		adcx3=Get_Adc(ADC_Channel_7 ,50)-2048; //PA7
// 		adcx4=Get_Adc(ADC_Channel_10,50)-2048; //PC0
// 		adcx5=Get_Adc(ADC_Channel_11,50)-2048; //PC1
// 		adcx6=Get_Adc(ADC_Channel_12,50)-2048; //PC2
// 		adcx7=Get_Adc(ADC_Channel_13,50)-2048; //PC3
}























