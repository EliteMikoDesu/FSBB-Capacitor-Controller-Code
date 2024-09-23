#include "stm32f3xx_hal.h"
#include "PID.h"
pid_type_def control_struct;
float LimitMax(float input,float max) 
{
    if (input > max)         
    {                       
			input = max;           
    }                        
    else if (input < -max)   
    {                        
      input = -max;          
    }     
return 		input;
}

void PID_init(pid_type_def *pid,  float p,  float i,  float d , float max_out, float max_iout,float min_out, float min_iout) 
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
	  pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->min_out = min_out;
    pid->min_iout = min_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set) 
{
	pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
	pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
  pid->Iout = pid->Ki * pid->error[0];
  pid->Dbuf[2] = pid->Dbuf[1];
  pid->Dbuf[1] = pid->Dbuf[0];
  pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
  pid->Dout = pid->Kd * pid->Dbuf[0];
  pid->out += pid->Pout + pid->Iout + pid->Dout;
	if(pid->out<pid->min_out)
		pid->out=pid->min_out;
	pid->out=LimitMax(pid->out,pid->max_out);
	return pid->out;
}

void PID_clear(pid_type_def *pid) 
{
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

