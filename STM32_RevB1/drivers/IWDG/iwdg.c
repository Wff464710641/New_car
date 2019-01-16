/***************************************************************
函数名称：独立看门狗
作者：流星落泪
时间：20150818
*************************************************************/
#include "iwdg.h"

void IWDG_Init(void) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写操作
	
	IWDG_SetPrescaler(4);  //设置IWDG预分频值:设置IWDG预分频值为64
	
	IWDG_SetReload(6);  //设置IWDG重装载值   时间*1.6=喂狗时间   8ms内喂狗一次 
	
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable();  //使能IWDG
}




