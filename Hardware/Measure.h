#ifndef __MEASURE_H
#define __MEASURE_H
#include "Power_Control.h"
typedef struct
{	
	double real_valu1;      //��ʵֵ1 �ⲿ�Ǳ���
	double get_volt1;       //����ֵ1  ��Ƭ���˲�����
	double real_valu2;      //��ʵֵ2
	double get_volt2;       //����ֵ2
	
	double real_valu3;      //��ʵֵ1 �ⲿ�Ǳ���
	double get_volt3;       //����ֵ1  ��Ƭ���˲�����
	double real_valu4;      //��ʵֵ2
	double get_volt4;       //����ֵ2
	
	float offset;          //Ư��ϵ��
	float ratio;           //����ϵ��
	
	int16_t measured_value; //ԭʼADCֵ
	int16_t bias;							//ԭʼƫ����
	float Solved_value;//��������ĵ�ѹ�͵���
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
float ADCvalue_to_ELEC(ELEC_INFO_STRUCT *p,uint16_t cn_kalman);//ADCֵ��ɵ�ѹ����
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
