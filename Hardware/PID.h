#ifndef __PID_H
#define __PID_H



typedef struct
{
    unsigned char mode;
    float Kp;
    float Ki;
    float Kd;
    float T;

    float max_out;
    float min_out;
    float max_iout;
    float min_iout;
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];

} pid_type_def;
float PID_calc(pid_type_def *pid, float ref, float set);
void PID_init(pid_type_def *pid,  float p,  float i,  float d , float max_out, float max_iout,float min_out, float min_iout) ;
float LimitMax(float input,float max) ;
void PID_clear(pid_type_def *pid) ;

#endif
