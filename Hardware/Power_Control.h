#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include "PID.h"
#include "Measure.h"
#include "math.h"



#define HRTIMMaster_Period 5760

#define BUCK_RIGHT_DUTY (uint16_t)(0.95 * HRTIMMaster_Period+50)    					//Buck模式下，右桥固定占空比95%
#define BUCK_LEFT_MIN_DUTY (uint16_t)(0.10 * HRTIMMaster_Period+50)   		    //Buck模式下，左桥最小占空比10%
#define BUCK_LEFT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)   			  //Buck模式下，左桥最大占空比90%

#define	BUCK_BOOST_LEFT_DUTY (uint16_t)(0.60 * HRTIMMaster_Period+50)   			//BUCK-BOOST模式下，左桥固定占空比60%，给太大放不了电
#define BUCK_BOOST_RIGHT_MIN_DUTY (uint16_t)(0.50 * HRTIMMaster_Period+50)    //BUCK-BOOST模式下，右桥最小占空比50%，给太大充不了电
#define BUCK_BOOST_RIGHT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)    //BUCK-BOOST模式下，右桥最大占空比90%

#define BOOST_LEFT_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)              //BOOST模式下，左桥固定占空比90%
#define BOOST_RIGHT_MIN_DUTY (uint16_t)(0.10 * HRTIMMaster_Period+50)         //BOOST模式下，右桥最小占空比10%
#define BOOST_RIGHT_MAX_DUTY (uint16_t)(0.90 * HRTIMMaster_Period+50)         //BOOST模式下，右桥最大占空比90%

#define V_CAP_preCharge 3.0                                                   //电容组恒流预充电电压

#define MIN_REG_VALUE   (uint16_t)25                     //HRTIM reg mini value

enum LOOP_MODE
{
    LOOP_MODE_CV = 1,                                       //恒压模式
    LOOP_MODE_CC                                            //恒流模式
};

enum CAP_MODE
{
    BUCK=1,       //BUCK模态
    BUCK_BOOST,   //BUCK-BOOST模态
    BOOST,        //BOOST模态
		Standby,      //故障或待机
    Shutdown
};

typedef struct {
  pid_type_def powerin_loop, currout_loop, voltout_loop;  //pid结构体
  float P_set;                                        //设定电管输出功率
	float I_Set;																						//电管输出电流
  float dcdc_power;                                       //DC-DC设定功率
  float I_Charge_limited;                                        //DC-DC设定电流
  float dcdc_max_curr;                                    //dcdc最大电流
  float cap_v_max;                                        //电容最大电压
  float cap_max_curr;                                     //电容最大电流
  float vloop_ratio;                                      //电压环输出
  float cloop_ratio;                                      //电流环输出
  float volt_ratio;                                       //最终输出电压比值 (Vout / Vin)
	enum	CAP_MODE Cap_Mode;																//超电状态
	enum	LOOP_MODE Loop_Mode;															//PID环状态
	int16_t left_duty;																			//左桥占空比
	int16_t right_duty;																			//右桥占空比
	
	int16_t buck_left_feedforward_duty;																			//BUCK左桥前馈占空比
	int16_t buck_boost_right_feedforward_duty;																			//BUCK-BOOST右桥前馈占空比	

	uint8_t BBModeChange;   																//工作模式切换标志位
	uint8_t flag;
	
	float I_DCDC_IN_MAX;																				//H桥左侧最大输入电流
	float I_CAP_IN_MAX;																					//H桥右侧最大输入电流
	float I_DCDC_OUT_MAX;																				//H桥左侧输出最大电流
	float I_CAP_OUT_MAX;																					//H桥右侧输出最大电流
  uint16_t STBYCnt;
  
} control_struct_t;

extern control_struct_t control; 




#endif
