#include "stm32f3xx_hal.h"
#include "Key.h"
#include "hrtim.h"
#include "Power_Control.h"
GPIO_PinState KEY1;
GPIO_PinState KEY2;
GPIO_PinState KEY3;
extern uint16_t CHA1,CHA2,CHB1,CHB2;
extern control_struct_t control; 
extern float pwm_1;
extern float V_Set;
/*------------------------------------------------------------------------------------------------------
����    �����������Ժ���1
����    �ܡ�                
����    ����
���� �� ֵ��
��ע�����                         
-------------------------------------------------------------------------------------------------------*/

void my_key()
{
	KEY1=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
	KEY2=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
	KEY3=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	static uint8_t Key_Flag_1=1;
	static uint8_t Key_Flag_2=1;  
	static uint8_t Key_Flag_3=1;//�������ɿ���־	  

	
	if(Key_Flag_1&&(KEY1==0))
	{
		HAL_Delay(10);
		Key_Flag_1=0;
		
		if(KEY1==0)      
		{
			control.P_set+=40;
		}
	} 
	if(KEY1==1) 
		Key_Flag_1=1; 			
	
	if(Key_Flag_2&&(KEY2==0))
	{
		HAL_Delay(10);
		Key_Flag_2=0;	
		
		if(KEY2 == 0)
		{
			control.P_set-=40;
			if(control.P_set<=0)
			{
				control.P_set=0;
			}
		}
	}			
	if(KEY2==1) Key_Flag_2=1; 					

	
	if(Key_Flag_3&&(KEY3==0))
	{	
		HAL_Delay(10);  //ȥ���� 
		Key_Flag_3=0;	//����һ�ΰ����ͽ��������±�־��0,��ֹ�ظ�����
		if(KEY3 == 0)
		{
			control.STBYCnt=0;
			Power_on_Self_Test();
		}
	}
	if(KEY3==1) Key_Flag_3=1; 	//���ް������»��Ű����ɿ��󣬰�����־λ ������1��Ϊ�´ΰ���������׼��	
}
