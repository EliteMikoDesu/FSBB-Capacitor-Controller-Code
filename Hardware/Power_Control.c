#include "stm32f3xx_hal.h"
#include "Power_Control.h"
#include "hrtim.h"
#include "TIMER.h"
#include "Measure.h"

control_struct_t control;                               //控制结构体

#define shift_duty 0.9f                                    //最大占空比
#define shift_duty_2 2 * shift_duty                         //最大占空比 * 2
#define MAX_PWM_CMP (shift_duty * DP_PWM_PER)           //PWM最大比较值
#define MIN_PWM_CMP ((1 - shift_duty) * DP_PWM_PER)     //PWM最小比较值
#define DP_PWM_PER 5760                                     //定时器HRTIM周期
#define max_volt_ratio 1.2f                                 //最大电压比值
#define min_volt_ratio 0.1f                                 //最小电压比值
#define MIN_UVP_VAL    18.0f//18V欠压保护
#define MIN_UVP_VAL_RE 21.0f//21V欠压保护恢复
#define MAX_UnderVoltageCount 5                             //

float pwm=0;
float pwm_1;

float V_Set=23.8f;
extern uint16_t CHA1,CHA2,CHB1,CHB2;
uint16_t Undervoltage_cnt=0;
uint16_t UVPCnt = 0;
uint16_t RSCnt = 0;
uint16_t STBYCnt = 0;

void Catastrophic_Failure(void)
{
	
	STBYCnt++;
	
	while(STBYCnt==5)
	{
		STBYCnt=0;
		control.Cap_Mode=Shutdown;
		Error_Handler();
	}
}

void  PID_Control()
{
	if (control.BBModeChange)
    {
        PID_clear(&control.currout_loop);
        PID_clear(&control.voltout_loop);
        PID_clear(&control.powerin_loop);
    }
    
	switch (control.Cap_Mode)
    {
		case Standby:   //故障
		{
			PID_clear(&control.currout_loop);
			PID_clear(&control.voltout_loop);
			PID_clear(&control.powerin_loop);
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
			//Catastrophic_Failure();
			break;
		}
		
		case BUCK:   //BUCK状态
		{
			if (control.BBModeChange!=0)
			{
			PID_init(&control.currout_loop, 2.0f, 1.0f, 0.01f, BUCK_LEFT_MAX_DUTY-BUCK_LEFT_MIN_DUTY, 200, BUCK_LEFT_MIN_DUTY-BUCK_LEFT_MAX_DUTY, -200);     //DC-DC电流环
  			PID_init(&control.voltout_loop, 2.0f, 1.0f, 0.01f,BUCK_LEFT_MAX_DUTY-BUCK_LEFT_MIN_DUTY, 200, BUCK_LEFT_MIN_DUTY-BUCK_LEFT_MAX_DUTY, -200);     //电容电压环
			control.BBModeChange = 0;
			}
			PID_calc(&control.currout_loop, measure.I_DCDC, control.I_Charge_limited );	//DCDC电流环
			PID_calc(&control.voltout_loop, ADC_V_CAP.Solved_value, V_Set);//电容电压环
			
			if(measure.P_DCDC>0)//需要充电
			{
				if(control.currout_loop.out<control.voltout_loop.out)//选择电压环还是电流环
				{
					control.left_duty=control.currout_loop.out+control.buck_left_feedforward_duty;
				}
				else
				{
					control.left_duty=control.voltout_loop.out+control.buck_left_feedforward_duty;
				}
			}
			
			else//需要放电
			{
				control.left_duty=control.currout_loop.out+control.buck_left_feedforward_duty;
			}
				
			if(control.left_duty>BUCK_LEFT_MAX_DUTY)//限制最大最小值
			{
				control.left_duty=BUCK_LEFT_MAX_DUTY;
			}
			if(control.left_duty<BUCK_LEFT_MIN_DUTY)
			{
				control.left_duty=BUCK_LEFT_MIN_DUTY;
			}

			if(measure.I_DCDC<0.5f && (ADC_V_CAP.Solved_filter_out/ADC_VIN.Solved_filter_out)<0.125)
			{
				control.left_duty=720;
			}
			
			control.right_duty=BUCK_RIGHT_DUTY;//给右桥固定值
			break;
		}
		
		case BUCK_BOOST:   //BUCK-BOOST状态
		{
			if (control.BBModeChange!=0)
			{
				PID_init(&control.currout_loop, 2.2f, 2.5f, 0.01f, BUCK_BOOST_RIGHT_MAX_DUTY- BUCK_BOOST_RIGHT_MIN_DUTY, 200, BUCK_BOOST_RIGHT_MIN_DUTY-BUCK_BOOST_RIGHT_MAX_DUTY, -200);     //DC-DC电流环
				PID_init(&control.voltout_loop, 2.2f, 2.5f, 0.01f,BUCK_BOOST_RIGHT_MAX_DUTY- BUCK_BOOST_RIGHT_MIN_DUTY, 200, BUCK_BOOST_RIGHT_MIN_DUTY-BUCK_BOOST_RIGHT_MAX_DUTY, -200);      //电容电压环
				control.BBModeChange = 0;
			}
			PID_calc(&control.currout_loop, measure.I_DCDC, control.I_Charge_limited );			//DCDC电流环
			PID_calc(&control.voltout_loop, ADC_V_CAP.Solved_value, V_Set);//电容电压环

			if(measure.P_DCDC>0)//需要充电
			{
				if(control.currout_loop.out<control.voltout_loop.out)//选择电压环还是电流环
				{
					control.right_duty=control.buck_boost_right_feedforward_duty-control.currout_loop.out;
				}
				else
				{
					control.right_duty=control.buck_boost_right_feedforward_duty-control.voltout_loop.out;
				}
			}
			
			else//需要放电
			{
				control.right_duty=control.buck_boost_right_feedforward_duty-control.currout_loop.out;
			}

			if(control.right_duty>BUCK_BOOST_RIGHT_MAX_DUTY)//限制最大最小值
			{
				control.right_duty=BUCK_BOOST_RIGHT_MAX_DUTY;
			}
			
			if(control.right_duty<BUCK_BOOST_RIGHT_MIN_DUTY)
			{
				control.right_duty=BUCK_BOOST_RIGHT_MIN_DUTY;
			}
			
			control.left_duty=BUCK_BOOST_LEFT_DUTY;//给左桥固定值		
			break;
		}

		case BOOST:
		{
			if (control.BBModeChange!=0)
			{
				PID_init(&control.currout_loop, 2.0f, 2.5f, 0.01f, BOOST_RIGHT_MAX_DUTY-BOOST_RIGHT_MIN_DUTY, 200, BUCK_LEFT_MIN_DUTY-BUCK_LEFT_MAX_DUTY, -200);     //DC-DC电流环
				PID_init(&control.voltout_loop, 2.0f, 2.5f, 0.01f,BUCK_LEFT_MAX_DUTY-BUCK_LEFT_MIN_DUTY, 200, BUCK_LEFT_MIN_DUTY-BUCK_LEFT_MAX_DUTY, -200);      //电容电压环
				control.BBModeChange = 0;
			}
		}
	}
		
	HRTIM1->sMasterRegs.MCMP1R=MIN_REG_VALUE;
	HRTIM1->sMasterRegs.MCMP3R=MIN_REG_VALUE;
		
	HRTIM1->sMasterRegs.MCMP2R=control.left_duty+MIN_REG_VALUE;
	HRTIM1->sMasterRegs.MCMP4R=control.right_duty+MIN_REG_VALUE;
}

void Power_on_Self_Test()
{
	HAL_Delay(1);
	
	for(int i=0;i<8;i++)
	{
		ADC_VIN.measured_array[i]=ADC_VIN.measured_value;
		ADC_I_IN.measured_array[i]=ADC_I_IN.measured_value;
		ADC_I_MOTOR.measured_array[i]=ADC_I_MOTOR.measured_value;
			
		ADC_V_CAP.measured_array[i]=ADC_V_CAP.measured_value;
		ADC_I_CAP.measured_array[i]=ADC_I_CAP.measured_value;
		HAL_Delay(1);
	}

	ADC_VIN.filter_out=ringbuf_cal(ADC_VIN.measured_array,8);
	ADC_I_IN.filter_out=ringbuf_cal(ADC_I_IN.measured_array,8);
	ADC_I_MOTOR.filter_out=ringbuf_cal(ADC_I_MOTOR.measured_array,8);
	
	ADC_V_CAP.filter_out=ringbuf_cal(ADC_V_CAP.measured_array,8);
	ADC_I_CAP.filter_out=ringbuf_cal(ADC_I_CAP.measured_array,8);
	
	ADC_VIN.Solved_filter_out=ADCvalue_to_ELEC(&ADC_VIN,ADC_VIN.filter_out);
	ADC_I_IN.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_IN,ADC_I_IN.filter_out);
	ADC_I_MOTOR.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_MOTOR,ADC_I_MOTOR.filter_out);
	
	ADC_V_CAP.Solved_filter_out=ADCvalue_to_ELEC(&ADC_V_CAP,ADC_V_CAP.filter_out);
	ADC_I_CAP.Solved_filter_out=ADCvalue_to_ELEC(&ADC_I_CAP,ADC_I_CAP.filter_out);

	
	if(ADC_VIN.Solved_filter_out < 20.0f||ADC_VIN.Solved_filter_out>28.0f)          //过压欠压
	{
		control.Cap_Mode=Standby;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	}
		
	if(ADC_I_IN.Solved_filter_out>0.5f||ADC_I_MOTOR.Solved_filter_out>0.5f||ADC_I_CAP.Solved_filter_out>0.5f)		//电流检测异常
	{
		control.Cap_Mode=Standby;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);			
	}
		
	control.vloop_ratio=ADC_V_CAP.Solved_filter_out/ADC_VIN.Solved_filter_out;
		
	if(control.vloop_ratio<0.9)
	{
		control.Cap_Mode=BUCK;
		control.buck_left_feedforward_duty=BUCK_RIGHT_DUTY*control.vloop_ratio;
		control.left_duty=control.buck_left_feedforward_duty;
		control.buck_boost_right_feedforward_duty=BUCK_RIGHT_DUTY;
			
		if(control.left_duty<BUCK_LEFT_MIN_DUTY)
		{
			control.left_duty=BUCK_LEFT_MIN_DUTY;
		}
		
		if(control.left_duty>BUCK_LEFT_MAX_DUTY)
		{
			control.left_duty=BUCK_LEFT_MAX_DUTY;
		}
			
		control.right_duty=BUCK_RIGHT_DUTY;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	}
		
	else if((0.9<=control.vloop_ratio)&&(control.vloop_ratio<1.2))
	{
		control.Cap_Mode=BUCK_BOOST;
		control.left_duty=BUCK_BOOST_LEFT_DUTY;
		control.buck_boost_right_feedforward_duty=BUCK_BOOST_LEFT_DUTY/control.vloop_ratio;
		control.buck_left_feedforward_duty=BUCK_BOOST_LEFT_DUTY;
		control.right_duty=control.buck_boost_right_feedforward_duty;
			
		if(control.right_duty<BUCK_BOOST_RIGHT_MIN_DUTY)
		{
			control.right_duty=BUCK_BOOST_RIGHT_MIN_DUTY;
		}
			
		if(control.right_duty>BUCK_BOOST_RIGHT_MAX_DUTY)
		{
			control.right_duty=BUCK_BOOST_RIGHT_MAX_DUTY;
		}
			
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	}

	/* else if((control.vloop_ratio>=1.2))
	{
		control.Cap_Mode=BOOST;
	}
	*/
	else
	{
		control.Cap_Mode=Standby;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	}
		
	HRTIM1->sMasterRegs.MCMP1R=MIN_REG_VALUE;
	HRTIM1->sMasterRegs.MCMP3R=MIN_REG_VALUE;
		
	HRTIM1->sMasterRegs.MCMP2R=control.left_duty+MIN_REG_VALUE;
	HRTIM1->sMasterRegs.MCMP4R=control.right_duty+MIN_REG_VALUE;
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //通道打开
}

void Input_undervoltage_protection()
{
	if(ADC_VIN.Solved_value<MIN_UVP_VAL)
	{
		UVPCnt++;
		if(UVPCnt>=5000)
		{
			UVPCnt=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			control.Cap_Mode=Standby;
		}
	}
	
	else
	{
		UVPCnt=0;
	}
	
}

void Mode_Judgment(void)
{
    //上一次模式状态量
    uint8_t PreBBFlag = 0;

    //暂存当前的模式状态量
    PreBBFlag = control.Cap_Mode;
    switch (control.Cap_Mode)
    {
    //NA
		case Standby:
		{
			control.STBYCnt++;
			
			if(control.STBYCnt>=MAX_UnderVoltageCount)
			{
				control.STBYCnt=MAX_UnderVoltageCount;
				break;
			}
			
			if (ADC_V_CAP.Solved_value < (ADC_VIN.Solved_value* 0.8f)) //vout<0.8*vin
			{
				control.Cap_Mode = BUCK; //buck mode
			}
			else if(ADC_V_CAP.Solved_value >= (ADC_VIN.Solved_value* 0.8f))
			{
				control.Cap_Mode = BUCK_BOOST; //buck-boost mode
			}

			break;
		}
			// BUCK模式
		case BUCK:
		{
				if((control.left_duty>=BUCK_LEFT_MAX_DUTY)&&(control.vloop_ratio<0.9))
				{
					control.Cap_Mode = BUCK_BOOST;
				}
			break;
		}
			//BUCK_BOOST模式
		case BUCK_BOOST:
		{
				if((control.I_Charge_limited>0)&&((0.9<=control.vloop_ratio)&&(control.vloop_ratio<1.2)))//充电模式
				{
					if (control.right_duty <= BUCK_BOOST_RIGHT_MIN_DUTY) //vout<0.8*vin
					{
						control.Cap_Mode = BUCK; //buck mode
					}
				}
				else
				{
					if (control.right_duty >= BUCK_BOOST_RIGHT_MAX_DUTY) //vout<0.8*vin
					{
						control.Cap_Mode = BUCK; //buck mode				
					}
			}
			break;
		}
    }
    //when mode changes,set reg
    if (PreBBFlag != control.Cap_Mode)
    control.BBModeChange = 1;
}


