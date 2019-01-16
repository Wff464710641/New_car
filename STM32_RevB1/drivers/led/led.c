
#include "main.h"
#include "key.h"
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
/**************************实现函数**********************************************
*功    能:		定义led引脚
*********************************************************************************/
void led_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
 //主板LED1端口初始	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;				 //LED0-->PC.2 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_ResetBits(GPIOC,GPIO_Pin_2);						 //PB.8 输出低电平
//蜂鸣器端口初始  副班语音GPIOE3
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //LED0-->PC.2 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_ResetBits(GPIOE,GPIO_Pin_3);	
}
/**************************程序结束*********************************************/

/**************************************************************************
函数功能：速度计算函数
入口参数：编码器返回值
返回  值：无
作者：流星落泪
库版本：V3.5
函数描述：利用编码器返回的值计算当前速度
**************************************************************************/

void speed(int values)
{
	static int add = 0;  //定义一个累计临时变量
	static int addi=0;
   if(values<20&&values>0) addi = 100; 
	if(values<190&&values>140) addi = 15; 
	if(values<360&&values>310) addi = 8; 
	if(values<470&&values>420) addi = 6; 
	if(values<690&&values>630) addi = 4; 
	else
	{
		addi = (690-values)/10;
	}
	if(++add >= addi)     //5ms进入函数加加一次  当超过参数值
	{	
     add = 0;		 //累计值重新赋值
	  YJ4=!YJ4;     //液晶屏霍尔信号翻转  使得速度显示
	}
}



