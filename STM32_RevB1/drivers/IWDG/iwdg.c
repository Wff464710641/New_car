/***************************************************************
�������ƣ��������Ź�
���ߣ���������
ʱ�䣺20150818
*************************************************************/
#include "iwdg.h"

void IWDG_Init(void) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	
	IWDG_SetPrescaler(4);  //����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64
	
	IWDG_SetReload(6);  //����IWDG��װ��ֵ   ʱ��*1.6=ι��ʱ��   8ms��ι��һ�� 
	
	IWDG_ReloadCounter();  //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	
	IWDG_Enable();  //ʹ��IWDG
}




