#include "main.h"
#include "adc.h"

/**************************************************************************
作者：流星落泪
库版本：v3.5
**************************************************************************/

/**************************************************************************
函数功能：ACD初始化
入口参数：无
返回  值：无
作    者：流星落泪
**************************************************************************/
void  Adc_Init(void)
{    

	ADC_InitTypeDef ADC_InitStructure; //adc结构体定义
	GPIO_InitTypeDef GPIO_InitStructure;//gpio结构体定义
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA,ENABLE);//使能 gpioa adc1 时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置adc的分频因子  adc最大是14m
	
 	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//设置引脚模式  模拟输入
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始PA4--PA7
	
	ADC_DeInit(ADC1);   //将外设ADC1全部寄存器设置缺省   复位ADC1
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;//设置ADC独立通道
	ADC_InitStructure.ADC_ScanConvMode=DISABLE; //单通道模式  扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;//连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//设置软件启动
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//设置数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel=8;//通道数
	ADC_Init(ADC1,&ADC_InitStructure);//初始ADC1
	
	ADC_Cmd(ADC1,ENABLE);//使能ADC1
	ADC_ResetCalibration(ADC1);  //开启adc1校准
	while(ADC_GetResetCalibrationStatus(ADC1));//等待校准结束
	ADC_StartCalibration(ADC1); //开启ADC1校准
	while(ADC_GetCalibrationStatus(ADC1));//等待校准结束
	delay_ms(1);
	Get_battery_volt();   
}		

/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
作    者：流星落泪
**************************************************************************/
u16 Get_Adc(u8 ch,u8 times )   
{
// 	//设置转换序列	  		 
// 	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
// 	ADC1->SQR3|=ch;		  			    
// 	ADC1->CR2|=1<<22;       //启动规则转换通道 
// 	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
// 	return ADC1->DR;		//返回adc值	
	
	u32 temp_val=0;//50次转换结果的和
	u8 t;//临时变量
	ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);    //通道规则
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//使能软件转换
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待转换结束
	
	for(t=0;t<times;t++)  //循环累加50次
	{
		temp_val+=ADC_GetConversionValue(ADC1); //读取ADC上次转换的值 进行50次累加
	}
	return temp_val/times;//求出来50次的平均值
}

/**************************************************************************
函数功能：读取电池电压
入口参数：无
返回  值：无
作者：流星落泪
库版本：V3.5
**************************************************************************/
void Get_battery_volt(void)   
{  
//以下ADC采集通道用哪个打开那个就可以
		Voltage = Get_Adc(ADC_Channel_4 ,50);           			//PA4 	电流采集  1638≈40A
																													//PA4		电压采集	24V/11 = 2.1818 / 3.3 * 4096 =  0x0a94
		main_current = Get_Adc(ADC_Channel_5 ,5);         		//PA5		0A = 2.5V  0x060f 40mV/A 15A=600mV 2.5V + 0.6V = 3.1V/2(10K + 10K) adc = 0x0783

		
//		temp7 = (Get_Adc(ADC_Channel_6 ,50)-2048)/2.7;   	//PA6  转向转向电位器电压邋AD采集
//		temp7 = Get_Adc(ADC_Channel_6 ,50);   							//PA6  转向转向电位器电压邋AD采集
// 		adcx3=Get_Adc(ADC_Channel_7 ,50)-2048; //PA7
// 		adcx4=Get_Adc(ADC_Channel_10,50)-2048; //PC0
// 		adcx5=Get_Adc(ADC_Channel_11,50)-2048; //PC1
// 		adcx6=Get_Adc(ADC_Channel_12,50)-2048; //PC2
// 		adcx7=Get_Adc(ADC_Channel_13,50)-2048; //PC3
}























