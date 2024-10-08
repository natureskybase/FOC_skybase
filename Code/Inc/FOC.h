#ifndef __FOC_H
#define __FOC_H
#include "main.h"
#include "math.h"
#include "string.h"
#include "pid.h"

#define _PI_2  1.57079632679
#define _PI    3.14159265359
#define _3PI_2 4.71238898038
#define _2PI   6.28318530718
#define _round(x) ((x) >= 0 ? (uint16_t)((x) + 0.5) : (uint16_t)((x) - 0.5))

#define sqr3        1.732050808
#define one_sqr3    0.5773502692
#define pi          3.1415926

#define DutyAmplitude   10000   //占空比的幅值，即ARR
#define VBAT            12      //输入电压 V
#define I_to_V          7

#define StanderedVoltage    2.9
#define AdcCycle            4096
#define AdcVoltageOffset    2048    //AdcCycle/2
#define SampleResistor      0.01    //单位是欧姆
#define OperationalAmplify  20      //运放放大倍率       


#define Angle_read()            as5047_read_angle()
#define PoleNum                 7        //极对数
#define MechanicalAngleOffset   35    //单位/度 ，在多级对的情况下，存在多个零位偏置(多个电角度零位)，取其中一个就行  35
#define EncoderCycle            8191    //2^14



typedef void (*info_get) (void*);
typedef void (*PID) (void*);

typedef struct MOTOR
{
    uint8_t ID;
    uint8_t spin_direction; //0停转 1正转 2反转

    float voltage_info[3];
    float voltage_control[3];
    float current_q;
    float current_d;
    float current_alpha;
    float current_beta;
    float voltage_q;
    float voltage_d;
    float voltage_alpha;
    float voltage_beta;

    uint16_t angle_raw;
    float machanical_theta;
    float theta;

    uint8_t foc_sector;

}MOTOR;

extern float U_bias[3];
extern MOTOR* FOC_MOTOR;
extern float DA,DB,DC;
extern uint8_t current_judge;


float _sin ( float a );
float _cos ( float a );

void FOC_init(MOTOR* motor);
void Current_get_init();
void Theta_get(MOTOR* motor);
void Current_get(MOTOR* motor);

void Clarke_transfrom(MOTOR* motor);
void Park_transfrom(MOTOR* motor);
void ParkAnti_transfrom(MOTOR* motor,float Uq,float Ud, float theta);
void Svpwm(MOTOR*motor, float Uq, float Ud, float angle_el);
void Svpwm_sensor(float Uref, float angle_el);
void Velocity_pid(MOTOR* motor, float q);

void Pwm_init(void);
void Pwm_change(float A, float B, float C);
// }FOC_Control;



#endif

