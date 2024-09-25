#include "stm32f3xx_hal.h"
#include "Led.h"
#include "Power_Control.h"
//PB12--G
//PB13--B
//PA14--G
//HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
//HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Led_Control(void)
{
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);
}
void Led_Config_Control(void)
{
	//if(control.currout_loop.out<control.voltout_loop.out)//C loop LED3 enable
	//if(measure.P_DCDC>0)//BUCK mode LED3 enable
	if(control.Cap_Mode==BUCK)//BUCK mode LED3 enable
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	}
	
	//else if(control.currout_loop.out>control.voltout_loop.out)//V loop LED2 enable
	//else if(measure.P_DCDC<=0)//BUCK-BOOST mode LED2 enable
	else if(control.Cap_Mode==BUCK_BOOST)//BUCK-BOOST mode LED2 enable
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}

	else if(control.Cap_Mode==Standby)//while STANBY both LEDs enable
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}
