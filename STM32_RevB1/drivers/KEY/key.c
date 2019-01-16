#include "key.h"
//#include "MiniBalance.h"
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化KEY0-->GPIOA.13,KEY1-->GPIOA.15  上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_11;//PING5-8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOG5 6 7 8
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;//PING5-8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOG5 6 7 8
} 
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：无
作    者：流星落泪
**************************************************************************/
void key(void)
{
	static u8 i=0; //临时延时变量
   	if(temp1==1)   //如果总启动开启
	{
			if(KEY0==0||KEY1==0||KEY2==0||KEY3==0||temp4==0)  //如果踏板开关按下 或蓝牙启动 
			{
				if(tabanyy1==0) YY1=0; //打开上车报警
				 
				if (temp5>=3)   //延时
				{
					
					temp9=1;
					Flag_Stop=0;  //启动
					
					tabanyy1=1;    //关闭上车报警变量
					if(++i>5) i=0,YY1=1;   //拉高报警引脚
				}
				
				if(KEY0==0||KEY1==0||KEY2==0||KEY3==0) temp4=1;  //当蓝牙启动时 踏板开关按下 蓝牙启动自动解除
			}
			else  //如果踏板开关没有按下 整车不启动
			{
				    
				    tabanyy1=0;   //重置上车报警变量值
				    YY1=1;  //拉高YY1 
				    temp9=0;
					Flag_Stop=1;	//关闭
			}
    }
	else  //如果遥控或者蓝牙没有启动  整车不启动
	{
		   
  	  		temp9=0;
 			Flag_Stop=1;	//关闭
	}

}









