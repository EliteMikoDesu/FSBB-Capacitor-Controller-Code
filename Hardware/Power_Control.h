#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include "PID.h"
#include "Measure.h"
#include "math.h"



#define HRTIMMaster_Period 5760

#define BUCK_RIGHT_DUTY (uint16_t)(0.95 * HRTIMMaster_Period+50)    					//Buckģʽ�£����Ź̶�ռ�ձ�95%
#define BUCK_LEFT_MIN_DUTY (uint16_t)(0.10 * HRTIMMaster_Period+50)   		    //Buckģʽ�£�������Сռ�ձ�10%
#define BUCK_LEFT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)   			  //Buckģʽ�£��������ռ�ձ�90%

#define	BUCK_BOOST_LEFT_DUTY (uint16_t)(0.60 * HRTIMMaster_Period+50)   			//BUCK-BOOSTģʽ�£����Ź̶�ռ�ձ�60%����̫��Ų��˵�
#define BUCK_BOOST_RIGHT_MIN_DUTY (uint16_t)(0.50 * HRTIMMaster_Period+50)    //BUCK-BOOSTģʽ�£�������Сռ�ձ�50%����̫��䲻�˵�
#define BUCK_BOOST_RIGHT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)    //BUCK-BOOSTģʽ�£��������ռ�ձ�90%

#define BOOST_LEFT_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)              //BOOSTģʽ�£����Ź̶�ռ�ձ�90%
#define BOOST_RIGHT_MIN_DUTY (uint16_t)(0.10 * HRTIMMaster_Period+50)         //BOOSTģʽ�£�������Сռ�ձ�10%
#define BOOST_RIGHT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)         //BOOSTģʽ�£��������ռ�ձ�90%

#define V_CAP_preCharge 3.0                                                   //���������Ԥ����ѹ

#define MIN_REG_VALUE   (uint16_t)25                     //HRTIM reg mini value

enum LOOP_MODE
{
    LOOP_MODE_CV = 1,                                       //��ѹģʽ
    LOOP_MODE_CC                                            //����ģʽ
};

enum CAP_MODE
{
    BUCK=1,       //BUCKģ̬
    BUCK_BOOST,   //BUCK-BOOSTģ̬
    BOOST,        //BOOSTģ̬
		Standby,      //���ϻ����
    Shutdown
};

typedef struct {
  pid_type_def powerin_loop, currout_loop, voltout_loop;  //pid�ṹ��
  float P_set;                                        //�趨����������
	float I_Set;																						//����������
  float dcdc_power;                                       //DC-DC�趨����
  float I_Charge_limited;                                        //DC-DC�趨����
  float dcdc_max_curr;                                    //dcdc������
  float cap_v_max;                                        //��������ѹ
  float cap_max_curr;                                     //����������
  float vloop_ratio;                                      //��ѹ�����
  float cloop_ratio;                                      //���������
  float volt_ratio;                                       //���������ѹ��ֵ (Vout / Vin)
	enum	CAP_MODE Cap_Mode;																//����״̬
	enum	LOOP_MODE Loop_Mode;															//PID��״̬
	int16_t left_duty;																			//����ռ�ձ�
	int16_t right_duty;																			//����ռ�ձ�
	
	int16_t buck_left_feedforward_duty;																			//BUCK����ǰ��ռ�ձ�
	int16_t buck_boost_right_feedforward_duty;																			//BUCK-BOOST����ǰ��ռ�ձ�	

	uint8_t BBModeChange;   																//����ģʽ�л���־λ
	uint8_t flag;
	
	float I_DCDC_IN_MAX;																				//H���������������
	float I_CAP_IN_MAX;																					//H���Ҳ�����������
	float I_DCDC_OUT_MAX;																				//H��������������
	float I_CAP_OUT_MAX;																					//H���Ҳ����������
  uint16_t STBYCnt;
  
} control_struct_t;

extern control_struct_t control; 




#endif
