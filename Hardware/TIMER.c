#include "stm32f3xx_hal.h"
#include "TIMER.h"
#include "tim.h"
#include "hrtim.h"
#include "Communication.h"
int TIM_Cnt=0;
int TIM_Flag=0;
extern uint16_t CHB2;

void HRTIM_Init(void)
{
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_A  );//主定时器     开关频率100K

//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //通道打开

}



