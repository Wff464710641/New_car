#ifndef __ADC_H
#define __ADC_H	
#include "main.h"
#include "stm32f10x_adc.h"
/**************************************************************************
作者：流星落泪
库版本：V3.5
**************************************************************************/
u16 Get_Adc(u8 ch,u8 times );

void Adc_Init(void);
void Get_battery_volt(void); 
 
#endif 















