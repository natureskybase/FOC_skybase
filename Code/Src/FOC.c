#include "foc.h"
#include "AS5600.h"
#include "dma.h"
#include "tim.h"



MOTOR FOC_MOTOR__;
MOTOR* FOC_MOTOR=&FOC_MOTOR__;

MOTOR_PID foc_pid_iq__;
MOTOR_PID* foc_pid_iq=&foc_pid_iq__;
MOTOR_PID foc_pid_id__;
MOTOR_PID* foc_pid_id=&foc_pid_id__;

float DA,DB,DC;
float d1,d2,d3,d4,d5,d6,d7;

uint16_t sine_array[200] = {
  0, 79, 158, 237, 316, 395, 473, 552, 631, 710,
  789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494,
  1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269,
  2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030,
  3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772,
  3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491,
  4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181,
  5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839,
  5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461,
  6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043,
  7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581,
  7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072,
  8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512,
  8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899,
  8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231,
  9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506,
  9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721,
  9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876,
  9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969,
  9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000
};

float _sin ( float a ) 
{
    if ( a < _PI_2 ) {
        return 0.0001 * sine_array[_round ( 126.6873 * a )];
    } else if ( a < _PI ) {
        return 0.0001 * sine_array[398 - _round ( 126.6873 * a )];
    } else if ( a < _3PI_2 ) {
        return -0.0001 * sine_array[-398 + _round ( 126.6873 * a )];
    } else {
        return -0.0001 * sine_array[796 - _round ( 126.6873 * a )];
    }
}

float _cos ( float a ) 
{
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin ( a_sin );
}

void Theta_get(MOTOR* motor)
{
    // MOTOR* info_motor = (struct MOTOR*)motor;
    uint16_t mechanical_angle = Angle_read();
    float theta = 0,theta_angle = 0;
    // if(mechanical_angle)
    // theta = (360*(mechanical_angle-MechanicalAngleOffset)/EncoderCycle) % PoleNum;
    // theta = (mechanical_angle % (EncoderCycle/PoleNum))*360.0/(EncoderCycle/PoleNum);
    theta_angle = 360.0*(mechanical_angle % (EncoderCycle/PoleNum))/(EncoderCycle/PoleNum) + MechanicalAngleOffset;//对应得到的是电角度
    // theta_angle = 360.0*(mechanical_angle % (EncoderCycle/PoleNum))/(EncoderCycle/PoleNum) - offset_angle;
    if(theta_angle <0)
    {
        theta_angle+=360;
    }
		if(theta_angle >360)
    {
        theta_angle-=360;
    }
	
    // theta_angle = 360 - theta_angle;
    theta = 1.0 * theta_angle*(pi/180.0);
    motor->angle_raw = mechanical_angle;
    motor->theta = theta;
    motor->machanical_theta = 360.0*mechanical_angle/EncoderCycle*(pi/180);

}

float U_bias[3];
#define bias_get(n)         1.0*(ADC_ConvertedValue[n]-AdcVoltageOffset)\
                            *StanderedVoltage/AdcCycle/SampleResistor/OperationalAmplify;
#define current_cal(n)      (1.0*(ADC_ConvertedValue[n]-AdcVoltageOffset)\
                            *StanderedVoltage/AdcCycle/SampleResistor/OperationalAmplify)-U_bias[n];
void Current_get_init()
{
    float U_bias_array[30]={0};
    float ans_a=0,ans_b=0,ans_c=0;
    for(uint16_t i=0;i<30;i++)
    {
        LL_mDelay(1);
        if(i<10)
        {
            U_bias_array[i]=bias_get(0);
            ans_a+=U_bias_array[i];
        }
        else if(i>=10 && i<20)
        {
            U_bias_array[i]=bias_get(1);
            ans_b+=U_bias_array[i];
        }
        else
        {
            U_bias_array[i]=bias_get(2);
            ans_c+=U_bias_array[i];
        }
    }
    U_bias[0]=ans_a/10;
    U_bias[1]=ans_b/10;
    U_bias[2]=ans_c/10;
}

uint8_t current_judge =0x00;
void Current_get(MOTOR* motor)
{
    static float V_A_last=0,V_B_last=0,V_C_last=0;
    static float V_A_last_last=0,V_B_last_last=0,V_C_last_last=0;
    float V_A, V_B, V_C;
    uint16_t _TIM_CCR1 = DA*DutyAmplitude, _TIM_CCR2 = DB*DutyAmplitude, _TIM_CCR3 = DC*DutyAmplitude;
    uint16_t _TIM_CNT = LL_TIM_GetCounter(TIM1);
    current_judge =0x00;
    
    current_judge |= ((_TIM_CNT>=_TIM_CCR1)<<2);
    current_judge |= ((_TIM_CNT>=_TIM_CCR2)<<1);
    current_judge |= (_TIM_CNT>=_TIM_CCR3);
    
    uint8_t sector =motor->foc_sector;


    // 0110
    if(current_judge == 0x06)
    {
        V_A = current_cal(0);
        V_B = current_cal(1);
        V_C = -V_A-V_B;
    }
    // 0101
    else if(current_judge == 0x05)
    {
        V_A = current_cal(0);
        V_C = current_cal(2);
        V_B = -V_A-V_C;
    }
    // 0011
    else if(current_judge == 0x03)
    {
        V_B = current_cal(1);
        V_C = current_cal(2);
        V_A = -V_B-V_C;   
    }
    // 0111
    else if(current_judge == 0x07)
    {
        V_A = current_cal(0);
        V_B = current_cal(1);
        V_C = current_cal(2);
    }
    else
    {
        return;
    }

    V_C = -V_A-V_B;

    // // 0.5uq - 0.5阈值
    // if(fabsf(V_A - V_A_last)-fabsf(V_A_last-V_A_last_last)>=1.8)V_A=V_A_last;
    // if(fabsf(V_B - V_B_last)-fabsf(V_B_last-V_B_last_last)>=1.8)V_B=V_B_last;
    // if(fabsf(V_C - V_C_last)-fabsf(V_C_last-V_C_last_last)>=1.8)V_C=V_C_last;
    
    // V_A = V_A*0.6+V_A_last*0.4;
    // V_B = V_B*0.6+V_B_last*0.4;
    // V_C = V_C*0.6+V_C_last*0.4;

    // V_A_last_last = V_A_last; V_A_last = V_A;
    // V_B_last_last = V_B_last; V_B_last = V_B;
    // V_C_last_last = V_C_last; V_C_last = V_C;
    
    motor->voltage_info[0] = V_A;
    motor->voltage_info[1] = V_C;
    motor->voltage_info[2] = V_B;
}

void current_reconstitution(MOTOR* motor)
{
    uint8_t sector = motor->foc_sector;
    // if(sector == )
}


void FOC_init(MOTOR* motor)
{
    memset(motor, 0, sizeof(struct MOTOR));

    motor->spin_direction = 1;//默认初始化为正方向
    PID_Init(foc_pid_iq,0.001, 0, 0, 0.1, 1.5);
    PID_Init(foc_pid_iq,0.001, 0, 0, 0.1, 1.5);
    /*PWM波初始化*/
    Pwm_init();


}


void Clarke_transfrom(MOTOR* motor)
{
    motor->current_alpha = 1.0*2/3*(motor->voltage_info[0] - motor->voltage_info[1]/2 -motor->voltage_info[2]/2);
    motor->current_beta = 1.0*one_sqr3*(motor->voltage_info[1] - motor->voltage_info[2]);
    
}

void Park_transfrom(MOTOR* motor)
{
    if(motor->spin_direction==2){
    motor->current_d = motor->current_alpha*_cos(motor->theta) + motor->current_beta*_sin(motor->theta);
    motor->current_q = motor->current_beta*_cos(motor->theta) - motor->current_alpha*_sin(motor->theta);
    }
    else if(motor->spin_direction==1){
    motor->current_d = motor->current_beta*_cos(motor->theta) + motor->current_alpha*_sin(motor->theta);
    motor->current_q = motor->current_alpha*_cos(motor->theta) - motor->current_beta*_sin(motor->theta);
    }
}


void ParkAnti_transfrom(MOTOR* motor,float Uq,float Ud, float theta)
{
    // motor->voltage_d = motor->current_d * I_to_V;
    // motor->voltage_q = motor->current_q * I_to_V;

    // motor->voltage_alpha = motor->voltage_d*cos(motor->theta) - motor->voltage_q*sin(motor->theta);
    // motor->voltage_beta = motor->voltage_d*sin(motor->theta) + motor->voltage_q*cos(motor->theta);

    // motor->voltage_alpha = Ud*cosf(theta) - Uq*sinf(theta);
    // motor->voltage_beta = Ud*sinf(theta) + Uq*cosf(theta);

    motor->voltage_alpha = Ud*_cos(theta) - Uq*_sin(theta);
    motor->voltage_beta = Ud*_sin(theta) + Uq*_cos(theta);
}

void Pwm_init(void)
{
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
    
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    
}

void Pwm_change(float A, float B, float C)
{
    TIM1->CCR1 = A * DutyAmplitude;
    TIM1->CCR2 = B * DutyAmplitude;
    TIM1->CCR3 = C * DutyAmplitude;
}


void Velocity_pid(MOTOR* motor, float q)
{

    Clarke_transfrom(motor);
    Park_transfrom(motor);

    float iq=motor->current_q;
    float id=motor->current_d;

    // PID_calc(foc_pid_iq, iq, q);
    PID_calc(foc_pid_id, id, 0);
    iq += foc_pid_iq->out;
    id += foc_pid_id->out;

    Svpwm(FOC_MOTOR,iq,id,FOC_MOTOR->theta);
    Pwm_change(DA,DB,DC);

}

void Svpwm(MOTOR*motor, float Uq, float Ud, float angle_el)
{   
    ParkAnti_transfrom(motor,Uq,Ud,angle_el);
    uint8_t sector=0;
    // uint8_t sector = (uint8_t)(angle_el / (pi/3)) + 1;    // find the sector we are in currently
    float V_alpha = motor->voltage_alpha;
    float V_beta = motor->voltage_beta;
    
    
    #pragma region {svpwm无感判断扇区}
    // float V_alpha_abs = fabs(V_alpha), V_beta_sqr3_abs = fabs(1.0*V_beta*one_sqr3);
    // if((V_alpha >= 0.0f) && (V_beta >=0.0f) && (V_alpha_abs >= V_beta_sqr3_abs))
    // {//foc_sector = 1
    //     d4 = 1.0 * (V_alpha - 1.0*V_beta*one_sqr3) / VBAT;
    //     d6 = 1.0 * (2.0*V_beta*one_sqr3) / VBAT;
    //     d7 = (1.0f - (d4 + d6)) * 0.5;

    //     DA = d4 + d6 + d7;
    //     DB = d6 + d7;
    //     DC = d7;

    //     motor->foc_sector = 1;
    // }

    // if((V_beta > 0.0f) && (V_alpha_abs <= 1.0*V_beta*one_sqr3))
    // {//foc_sector = 2
    //     d6 = 1.0 * (V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
    //     d2 = 1.0 * (-V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
    //     d7 = (1.0f - (d2 + d6)) * 0.5;

    //     DA = d6 + d7;
    //     DB = d6 + d7 + d2;
    //     DC = d7;

    //     motor->foc_sector = 2;
    // }

    // if((V_alpha <= 0.0f) && (V_beta >= 0.0f) && (V_alpha_abs >= V_beta_sqr3_abs))
    // {
    //     d2 = 1.0 * (2.0*V_beta*one_sqr3) / VBAT;
    //     d3 = 1.0 * (-V_alpha - 1.0*V_beta*one_sqr3) / VBAT;
    //     d7 = (1.0f - (d2 + d3)) * 0.5f;

    //     DA = d7;
    //     DB = d3 + d7 +d2;
    //     DC = d7 + d3;

    //     motor ->foc_sector = 3;
    // }

    // if((V_alpha <= 0.0f) && (V_beta <= 0.0f) && (V_alpha_abs >= V_beta_sqr3_abs))
    // {
    //     d1 = -1.0 * (2.0*V_beta*one_sqr3) / VBAT;
    //     d3 = 1.0 * (-V_alpha + 1.0*V_beta*one_sqr3) / VBAT;
    //     d7 = (1.0f - (d3 + d1)) * 0.5f;

    //     DA = d7;
    //     DB = d3 + d7;
    //     DC = d3 + d7 + d1;

    //     motor ->foc_sector = 4;
    // }

    // if((V_alpha_abs <= -1.0f * V_beta*one_sqr3))
    // {
    //     d1 = 1.0 * (-V_alpha - 1.0f * V_beta*one_sqr3) / VBAT;
    //     d5 = 1.0 * (V_alpha - 1.0f * V_beta*one_sqr3) /VBAT;
    //     d7 = (1.0f - (d1 + d5)) * 0.5f;

    //     DA = d5 + d7;
    //     DB = d7;
    //     DC = d1 + d5 + d7;

    //     motor->foc_sector = 5;
    // }
    
    // if((V_alpha >= 0.0f) && (V_beta <= 0.0f) && (V_alpha_abs >= V_beta_sqr3_abs))
    // {
    //     d4 = 1.0 * (V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
    //     d5 = -1.0 * (2.0*V_beta*one_sqr3) / VBAT;
    //     d7 = (1.0f - (d4 + d5)) * 0.5f;

    //     DA = d4 + d5 + d7;
    //     DB = d7;
    //     DC = d5 + d7;

    //     motor->foc_sector = 6;
    // }
   
    // Pwm_change(DA,DB,DC);
    #pragma endregion 
		
    #pragma region {svpwm废弃代码}
    // uint8_t sectionMap[7] = {0, 2, 6, 1, 4, 3, 5};
    // float U1, U2, U3;
    // uint8_t a, b, c, n = 0;
    // U1 = V_beta;
    // U2 = (sqr3 * V_alpha - V_beta) / 2;
    // U3 = (-sqr3 * V_alpha - V_beta) / 2;
    //  if(U1 > 0)
    //     a = 1;
    // else 
    //     a = 0;
    // if(U2 > 0)
    //     b = 1;
    // else 
    //     b = 0;
    // if(U3 > 0)
    //     c = 1;
    // else 
    //     c = 0;
    
    // n = (c << 2) + (b << 1) + a;
    
    // switch(sectionMap[n])
    // {
        
    //     case 1:
    //     {
    //         float t4 = sqr3 * 1 * U2 / VBAT;
    //         float t6 = sqr3 * 1 * U1 / VBAT;
    //         float t0 = (1 - t4 - t6) / 2;
            
    //         DA = t4 + t6 + t0;
    //         DB = t6 + t0;
    //         DC = t0;
    //     }break;
        
    //     case 2:
    //     {
    //         float t2 = -sqr3 * 1 * U2 / VBAT;
    //         float t6 = -sqr3 * 1 * U3 / VBAT;
    //         float t0 = (1 - t2 - t6) / 2;
            
    //         DA = t6 + t0;
    //         DB = t2 + t6 + t0;
    //         DC = t0;
    //     }break;
        
    //     case 3:
    //     {
    //         float t2 = sqr3 * 1 * U1 / VBAT;
    //         float t3 = sqr3 * 1 * U3 / VBAT;
    //         float t0 = (1 - t2 - t3) / 2;
            
    //         DA = t0;
    //         DB = t2 + t3 + t0;
    //         DC = t3 + t0;
    //     }break;
        
    //     case 4:
    //     {
    //         float t1 = -sqr3 * 1 * U1 / VBAT;
    //         float t3 = -sqr3 * 1 * U2 / VBAT;
    //         float t0 = (1 - t1 - t3) / 2;
            
    //         DA = t0;
    //         DB = t3 + t0;
    //         DC = t1 + t3 + t0;
    //     }break;
        
    //     case 5:
    //     {
    //         float t1 = sqr3 * 1 * U3 / VBAT;
    //         float t5 = sqr3 * 1 * U2 / VBAT;
    //         float t0 = (1 - t1 - t5) / 2;
            
    //         DA = t5 + t0;
    //         DB = t0;
    //         DC = t1 + t5 + t0;
    //     }break;
        
    //     case 6:
    //     {
    //         float t4 = -sqr3 * 1 * U3 / VBAT;
    //         float t5 = -sqr3 * 1 * U1 / VBAT;
    //         float t0 = (1 - t4 - t5) / 2;
            
    //         DA = t4 + t5 + t0;
    //         DB = t0;
    //         DC = t5 + t0;
    //     }break;
       
    // }
    #pragma endregion
		
    #pragma region {}
    uint8_t section[7] = {0, 2, 6, 1, 4, 3, 5};
    float U1, U2, U3;
    uint8_t a, b, c;
    U1 = V_beta;
    U2 = (sqr3 * V_alpha - V_beta) / 2;
    U3 = (-sqr3 * V_alpha - V_beta) / 2;
    
    if(U1>0)a=1;else a=0;
    if(U2>0)b=1;else b=0;
    if(U3>0)c=1;else c=0;
    sector = section[(c << 2)+(b << 1) + a];

    // sector += (sector > 2) ? -2:4; 
        
    motor->foc_sector = sector;

    if(sector==1)
    {//foc_sector = 1
        d4 = 1.0 * (V_alpha - 1.0*V_beta*one_sqr3) / VBAT;
        d6 = 1.0 * (2.0*V_beta*one_sqr3) / VBAT;
        d7 = (1.0f - (d4 + d6)) * 0.5f;

        DA = d4 + d6 + d7;
        DB = d6 + d7;
        DC = d7;
        
    
        motor->foc_sector = 1;
    }

    if(sector==2)
    {//foc_sector = 2
        d6 = 1.0 * (V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
        d2 = 1.0 * (-V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
        d7 = (1.0f - (d2 + d6)) * 0.5f;

        DA = d6 + d7;
        DB = d6 + d7 + d2;
        DC = d7;
        
        motor->foc_sector = 2;
    }

    if(sector==3)
    {
        d2 = 1.0 * (2.0*V_beta*one_sqr3) / VBAT;
        d3 = 1.0 * (-V_alpha - 1.0*V_beta*one_sqr3) / VBAT;
        d7 = (1.0f - (d2 + d3)) * 0.5f;

        DA = d7;
        DB = d3 + d7 +d2;
        DC = d7 + d3;
        
        motor ->foc_sector = 3;
    }

    if(sector==4)
    {
        d1 = -1.0 * (2.0*V_beta*one_sqr3) / VBAT;
        d3 = 1.0 * (-V_alpha + 1.0*V_beta*one_sqr3) / VBAT;
        d7 = (1.0f - (d3 + d1)) * 0.5f;

        DA = d7;
        DB = d3 + d7;
        DC = d3 + d7 + d1;
        
        motor ->foc_sector = 4;
    }

    if(sector==5)
    {
        d1 = 1.0 * (-V_alpha - 1.0f * V_beta*one_sqr3) / VBAT;
        d5 = 1.0 * (V_alpha - 1.0f * V_beta*one_sqr3) /VBAT;
        d7 = (1.0f - (d1 + d5)) * 0.5f;

        DA = d5 + d7;
        DB = d7;
        DC = d1 + d5 + d7;
        
        motor->foc_sector = 5;
    }
    
    if(sector==6)
    {
        d4 = 1.0 * (V_alpha + 1.0f*V_beta*one_sqr3) / VBAT;
        d5 = -1.0 * (2.0*V_beta*one_sqr3) / VBAT;
        d7 = (1.0f - (d4 + d5)) * 0.5f;

        DA = d4 + d5 + d7;
        DB = d7;
        DC = d5 + d7;

        motor->foc_sector = 6;
    }
    #pragma endregion{}
}


// simple_Foc
void Svpwm_sensor(float Uref, float angle_el)
{
  
  uint8_t sector = (uint8_t)(angle_el / (pi/3)) + 1;    // find the sector we are in currently
  float T1 = sqr3*sinf(sector*(pi/3) - angle_el) * Uref/VBAT;
  float T2 = sqr3*sinf(angle_el - (sector-1)*(pi/3)) * Uref/VBAT;
  float T0 = 1 - T1 - T2;
  FOC_MOTOR->foc_sector = sector;
  switch(FOC_MOTOR->foc_sector)
  {
    case 1:
      DA = T1 + T2 + T0/2;
      DB = T2 + T0/2;
      DC = T0/2;
      break;
    case 2:
      DA = T1 +  T0/2;
      DB = T1 + T2 + T0/2;
      DC = T0/2;
      break;
    case 3:
      DA = T0/2;
      DB = T1 + T2 + T0/2;
      DC = T2 + T0/2;
      break;
    case 4:
      DA = T0/2;
      DB = T1+ T0/2;
      DC = T1 + T2 + T0/2;
      break;
    case 5:
      DA = T2 + T0/2;
      DB = T0/2;
      DC = T1 + T2 + T0/2;
      break;
    case 6:
      DA = T1 + T2 + T0/2;
      DB = T0/2;
      DC = T1 + T0/2;
      break;
    // default:  // possible error state
    //   DA = 0;
    //   DB = 0;
    //   DC = 0;
  }

  
}


















