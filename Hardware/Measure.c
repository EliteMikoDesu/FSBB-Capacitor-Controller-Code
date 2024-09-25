#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"                  // Device header
#include "Measure.h"
#include "adc.h"
extern float V_Set;
uint16_t voltage_vrefint_proportion;
uint16_t AD_Buf_1[3];
uint16_t AD_Buf_2[3];

ELEC_INFO_STRUCT ADC_I_IN;
ELEC_INFO_STRUCT ADC_VIN;
ELEC_INFO_STRUCT ADC_I_MOTOR;
ELEC_INFO_STRUCT ADC_I_CAP;
ELEC_INFO_STRUCT ADC_V_CAP;
ELEC_INFO_STRUCT ADC_V_REFINT;
extern uint16_t LED_Cnt;
uint8_t board_number=1;

measure_struct_t measure; 

float V_Motor=0;
uint8_t ringbuf_cnt=0;

void ratio_init()
{

	ADC_I_IN.ratio = 1/0.25f*3.3f/4095;
	ADC_VIN.ratio = 16*3.3f/4095;
	ADC_I_MOTOR.ratio=1/0.25f*3.3f/4095;
	ADC_I_CAP.ratio = 1/0.25f*3.3f/4095;
	ADC_V_CAP.ratio=16*3.3f/4095;
	ADC_I_IN.bias = 2048;
	ADC_I_MOTOR.bias=2048;
	ADC_I_CAP.bias = 2048;
}

float ringbuf_cal(int16_t *adc_ch , int16_t wide)//加和所有ADC测量值并返回平均值
{
	int32_t Total_Adc=0;
	for(int j=0;j<wide;j++)
	{
		Total_Adc+=adc_ch[j];
	}
	uint16_t adc_tmp;
	adc_tmp=Total_Adc/wide;
	return adc_tmp;
}

void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2) //线性校准运行一次
{
	double a, b;
	p->get_volt1 = get1;
	p->real_valu1 = real1;
	p->get_volt2 = get2;
	p->real_valu2 = real2;
	if ((p->get_volt2 - p->get_volt1 == 0) || (p->real_valu2 - p->real_valu1 == 0))
	{
		return;
	}

	a = (p->real_valu2 - p->real_valu1) / (p->get_volt2 - p->get_volt1);
	b = (p->real_valu1 - p->get_volt1 * a);

	p->ratio = a * p->ratio;
	p->offset = b;
}

float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman)//ADC值变成电压电流
{
	float a;
    p->measured_value=cn_kalman;
    a = (p->measured_value) * p->ratio + p->offset ;
	return a;
}

void ADC_Measure(void)
{
	ADC_VIN.measured_value=AD_Buf_1[0];//接收从DMA发来的数据
	ADC_I_IN.measured_value=AD_Buf_1[1]-ADC_I_IN.bias;
	ADC_I_MOTOR.measured_value=AD_Buf_1[2]-ADC_I_MOTOR.bias;
	ADC_V_CAP.measured_value=AD_Buf_2[0];
	ADC_I_CAP.measured_value=AD_Buf_2[1]-ADC_I_CAP.bias;
	ADC_V_REFINT.measured_value=AD_Buf_2[2];
	
	ADC_VIN.measured_array[ringbuf_cnt]=ADC_VIN.measured_value;//创建数组储存20个ADC采样值
	ADC_I_IN.measured_array[ringbuf_cnt]=ADC_I_IN.measured_value;
	ADC_I_MOTOR.measured_array[ringbuf_cnt]=ADC_I_MOTOR.measured_value;
		
	ADC_V_CAP.measured_array[ringbuf_cnt]=ADC_V_CAP.measured_value;
	ADC_I_CAP.measured_array[ringbuf_cnt]=ADC_I_CAP.measured_value;
	ADC_V_REFINT.measured_array[ringbuf_cnt]=ADC_V_REFINT.measured_value;
	
	ringbuf_cnt++;
	
	if(ringbuf_cnt>=20)//测量20组ADC数据后进行模式判断
	{
		LED_Cnt++;
		control.flag=1;
		ringbuf_cnt=0;
	
		ADC_VIN.filter_out=ringbuf_cal(ADC_VIN.measured_array,20);//传递结果
		ADC_I_IN.filter_out=ringbuf_cal(ADC_I_IN.measured_array,20);
		ADC_I_MOTOR.filter_out=ringbuf_cal(ADC_I_MOTOR.measured_array,20);
		
		ADC_V_CAP.filter_out=ringbuf_cal(ADC_V_CAP.measured_array,20);
		ADC_I_CAP.filter_out=ringbuf_cal(ADC_I_CAP.measured_array,20);
		ADC_V_REFINT.filter_out=ringbuf_cal(ADC_V_REFINT.measured_array,20);
			
		//计算VDDA
		ADC_V_REFINT.Solved_value=3.3f*measure.V_REFINT_CAL/ADC_V_REFINT.filter_out;

		//计算电管输出功率
		ADC_I_IN.Solved_value=ADCvalue_to_ELEC(&ADC_I_IN,ADC_I_IN.filter_out);
		ADC_VIN.Solved_value=ADCvalue_to_ELEC(&ADC_VIN,ADC_VIN.filter_out)+ADC_I_IN.Solved_value/250;
		measure.P_In=ADC_I_IN.Solved_value*ADC_VIN.Solved_value;
			
		//计算输入底盘功率
		ADC_I_MOTOR.Solved_value=ADCvalue_to_ELEC(&ADC_I_MOTOR,ADC_I_MOTOR.filter_out);
		V_Motor=ADC_VIN.Solved_value-ADC_I_MOTOR.Solved_value/250;
		
		measure.P_Motor=ADC_I_MOTOR.Solved_value*V_Motor;
			
		//计算DC-DC输入/输出功率
		measure.V_DCDC = ADC_VIN.Solved_value;
		measure.I_DCDC = ADC_I_IN.Solved_value - ADC_I_MOTOR.Solved_value ;
		measure.P_DCDC = measure.V_DCDC * measure.I_DCDC ;
			
		//计算电容输入/输出功率
		ADC_V_CAP.Solved_value=ADCvalue_to_ELEC(&ADC_V_CAP,ADC_V_CAP.filter_out);
		ADC_I_CAP.Solved_value=ADCvalue_to_ELEC(&ADC_I_CAP,ADC_I_CAP.filter_out);
		
		measure.P_Cap=ADC_V_CAP.Solved_value*ADC_I_CAP.Solved_value;
			
		//计算DC-DC效率
		if(measure.I_DCDC>0)
		{
			measure.efficiency=measure.P_Cap/measure.P_DCDC*100;
		}
		else
		{
			measure.efficiency=measure.P_DCDC/measure.P_Cap*100;
		}
		
		//计算剩余能量
		if(control.left_duty!=BUCK_LEFT_MIN_DUTY)
		{
			measure.surplus_energy=(ADC_V_CAP.Solved_value*ADC_V_CAP.Solved_value)/(23.85f*23.85f)*100;
			if(measure.surplus_energy>100)
			{
				measure.surplus_energy=100;
			}
		}
		else
		{
			measure.surplus_energy=0;
		}

		//计算目标DC-DC输入电流
		control.I_Set = control.P_set / ADC_VIN.Solved_value ;
		if((control.powerin_loop.out / measure.V_DCDC)> control.I_Set)
		{
			if(control.I_Set>control.I_CAP_OUT_MAX)
			control.I_Charge_limited=control.I_Set;
			else
			control.I_Charge_limited=control.I_CAP_OUT_MAX;
		}
		else
		{
			if((control.powerin_loop.out / measure.V_DCDC)>control.I_CAP_OUT_MAX)
			control.I_Charge_limited=(control.powerin_loop.out / measure.V_DCDC);
			else
			control.I_Charge_limited=control.I_CAP_OUT_MAX;
		}
		
		//限制目标DC-DC输入电流
		control.I_CAP_IN_MAX=10.0f;
		control.I_CAP_OUT_MAX=-8.0f;
		control.I_DCDC_IN_MAX = ADC_V_CAP.Solved_value/measure.V_DCDC*control.I_CAP_IN_MAX;
		control.I_DCDC_OUT_MAX = ADC_V_CAP.Solved_value/measure.V_DCDC*control.I_CAP_OUT_MAX;
		
		if(control.I_Charge_limited>control.I_DCDC_IN_MAX)
		{
			control.I_Charge_limited=control.I_DCDC_IN_MAX;
		}

		if(control.I_Charge_limited<control.I_DCDC_OUT_MAX)
		{
			control.I_Charge_limited=control.I_DCDC_OUT_MAX;
		}
	}
}

void ADC_init(void)
{
	
	control.P_set=15.0;
	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);//ADC软件校准
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);//ADC软件校准
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AD_Buf_1,3);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&AD_Buf_2,3);
	ratio_init();
	switch(board_number)
	{
	
		case 1:
		{
		ADC_Linear_calibration_init(&ADC_VIN,24.0,23.5,22.0,21.54);
		ADC_Linear_calibration_init(&ADC_I_IN,1.08,1.03,5.15,5.02);
		ADC_Linear_calibration_init(&ADC_I_MOTOR,5,4.94,1,0.95);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
		ADC_Linear_calibration_init(&ADC_V_CAP,21.9f,21.5f,16.33f,16.0f);
		break;
		}
		case 3:
		{
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}	
		case 4:
		{
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}
		case 5:
		{	
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}
		case 6:
		{	
//		ADC_Linear_calibration_init(&ADC_I_IN,24.0,24.24,25.5,23.75);
//		ADC_Linear_calibration_init(&ADC_VIN,1.45,1.36,2.52,2.39);
//		ADC_Linear_calibration_init(&ADC_I_MOTOR,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_I_CAP,17.38,17.09,21.0,20.58);
//		ADC_Linear_calibration_init(&ADC_V_CAP,17.38,17.09,21.0,20.58);
		break;
		}		
	}
}
