#ifndef __PID__H_
#define __PID__H_

typedef struct MOTOR_PID
{
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //PID最大输出
    float max_iout; //PID最大积分输出

    float set;	  //PID目标值
    float ref;	  //PID当前值

    float out;		//三项叠加输出
    float Pout;		//比例项输出
    float Iout;		//积分项输出
    float Dout;		//微分项输出
    //微分项最近三个值 0最新 1上一次 2上上次
    float Dbuf[3];
    //误差项最近三个值 0最新 1上一次 2上上次
    float error[3];  

}MOTOR_PID;


void PID_Init(MOTOR_PID* pid, float kp, float ki,float kd, float iout_limit, float output_limit);
void PID_calc(MOTOR_PID* pid ,float ref, float set);    



#endif
