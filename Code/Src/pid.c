#include "pid.h"
#include "tim.h"

#define LimitMax(input, max)   \
{                          \
if (input > max)       \
{                      \
    input = max;       \
}                      \
else if (input < -max) \
{                      \
    input = -max;      \
}                      \
}

void PID_Init(MOTOR_PID* pid, float kp, float ki,
        float kd, float iout_limit, float output_limit)
{
    if(pid == NULL)
    return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_iout = iout_limit;
    pid->max_out = output_limit;
    
}

void PID_calc(MOTOR_PID* pid ,float ref, float set)
{
    float kp=pid->Kp;
    float ki=pid->Ki;
    float kd=pid->Kd;
    
    pid->set =set;
    pid->ref =ref;
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    // pid->error[2] = pid->error[1];
    // pid->error[1] =pid->error[0];

    pid->error[0] =set-ref;
    pid->Dbuf[0] = pid->error[0]-pid->error[1];

    pid->Pout = kp * pid->error[0];
    pid->Iout += ki * pid->error[0];
    pid->Dout = kd * pid->Dbuf[0];

    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Dout+ pid->Pout+ pid->Iout;
    // LimitMax(pid->out, pid->max_out);
}



