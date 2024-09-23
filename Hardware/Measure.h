#ifndef __MEASURE_H
#define __MEASURE_H
#include "Power_Control.h"
typedef struct
{	
	double real_valu1;      //真实值1 外部仪表获得
	double get_volt1;       //测量值1  单片机滤波后获得
	double real_valu2;      //真实值2
	double get_volt2;       //测量值2
	
	double real_valu3;      //真实值1 外部仪表获得
	double get_volt3;       //测量值1  单片机滤波后获得
	double real_valu4;      //真实值2
	double get_volt4;       //测量值2
	
	float offset;          //漂移系数
	float ratio;           //比例系数
	
	int16_t measured_value; //原始ADC值
	int16_t bias;							//原始偏移量
	float Solved_value;//解算出来的电压和电流
	int16_t measured_array[20];
	float filter_out;
	float Solved_filter_out;
	
} ELEC_INFO_STRUCT;

extern ELEC_INFO_STRUCT ADC_I_IN;
extern ELEC_INFO_STRUCT ADC_VIN;
extern ELEC_INFO_STRUCT ADC_I_MOTOR;
extern ELEC_INFO_STRUCT ADC_I_CAP;
extern ELEC_INFO_STRUCT ADC_V_CAP;
extern ELEC_INFO_STRUCT ADC_V_REFINT;
extern uint16_t AD_Buf_1[3];
extern uint16_t AD_Buf_2[3];


float ringbuf_cal(int16_t *adc_ch , int16_t wide);
void ratio_init(void);
void ADC_Linear_calibration_init(ELEC_INFO_STRUCT *p, double real1, double get1, double real2, double get2);
float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman);//ADC值变成电压电流
void ADC_Measure(void);
void ADC_init(void);

extern uint8_t board_number;



typedef struct 
	{
	
		float P_In;	
		float P_Motor;
		float P_Cap;
		float V_DCDC;
		float I_DCDC;
		float P_DCDC;
		float efficiency;
		float surplus_energy;
		uint16_t V_REFINT_CAL;
	
} measure_struct_t;
extern measure_struct_t measure; 

#endif
