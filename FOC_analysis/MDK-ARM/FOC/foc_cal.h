#ifndef FOC
#define FOC
/*************************************************/
#include "math.h"
#include "string.h"
#include "stdio.h" 
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void iabc_2_idq( float ia,float ib,float ic,float theta,float *id,float *iq);
void idq_2_iabc(float theta_,float vd_,float vq_, float *va, float *vb, float *vc);
float PI_controller(float ref, float fb, float kp, float ki, float sampletime);
void pwm (float Va, float Vb, float Vc);
void svpwm (float V_alpha, float V_beta);
int PIDVel(float DesiredValue, float CurrentValue, float kp,float ki,float kd, float err_reset );
/*************************************************/
#endif