#include "foc_cal.h"

//---------------------------------------------------------Park clark 1
void iabc_2_idq(float ia,float ib,float ic,float theta,float *id,float *iq) 
{
	//clark iabc to ialpha_beta
	float i_alpha = (2/3) * ( ia - (ib + ic)/2);
	float i_beta = (2/3) * ((sqrt(3)/2)*ib - (sqrt(3)/2)*ic );
	// park iab to idq
	*id = i_alpha*sinf(theta) - i_beta*cosf(theta);
	*iq = i_alpha*cosf(theta) + i_beta*sinf(theta);
}

//---------------------------------------------------------Park clark 2
void idq_2_iabc(float theta_,float vd_,float vq_, float *va, float *vb, float *vc) 
{
	//park idq to ialpha_beta
	float v_alpha =  vd_*sinf(theta_) + vq_*cosf(theta_);
	float v_beta  = -vd_*cosf(theta_) + vq_*sinf(theta_);
	// clark iab to iabc
	*va = v_alpha; 
	*vb = (-1/2)*v_alpha + (sqrt(3)/2)*v_beta;
	*vc = (-1/2)*v_alpha - (sqrt(3)/2)*v_beta;
}

//---------------------------------------------------------PI
float PI_controller(float ref, float fb, float kp, float ki, float sampletime)
{
	float err = 0;
	float up,ui,out;
	float ui_p = 0; // chú ý
	
	err = ref - fb;
	up = kp * err;
	ui = ui_p + ki * err * sampletime;
	out = (up + ui);
	return out;
}

//---------------------------------------------------------PI + anti windup
int PIDVel(float DesiredValue, float CurrentValue, float kp,float ki,float kd, float err_reset )
{
	static float err_p = 0;
	static float ui_p = 0;
	float err,up,ud,ui;
	
	int HILIM = 100;
	int LOLIM = 0;
	float uout;
	float sampletime = 0.005;
	float kb = 0;
	float uhat;
	
	err = DesiredValue - CurrentValue;
	up = kp * err;
	ud = kd * (err-err_p) / sampletime;
	ui = ui_p + ki * err * sampletime + kb * sampletime * err_reset;
	
	err_p = err;
	ui_p = ui;
	
	uout = (int)(up+ud+ui);
	if(uout > HILIM)
	{
		uout = HILIM;
	}
	else if (uout < LOLIM)
		uout = LOLIM;
	else uhat = uout;
	err_reset = uhat - uout;
	return uout;
}
//---------------------------------------------------------tinh PWM tu vabc
void pwm (float Va, float Vb, float Vc){
	//scale gia tri ve 0-1
	float VDC = 24;
	float PWM_MAX = 100;
	float duty_a = (Va / VDC) + 0.5f;
  float duty_b = (Vb / VDC) + 0.5f;
  float duty_c = (Vc / VDC) + 0.5f;
  // Kiem tra co vot lo hay khong
  if (duty_a > 1.0f) duty_a = 1.0f;
  if (duty_a < 0.0f) duty_a = 0.0f;
  if (duty_b > 1.0f) duty_b = 1.0f;
  if (duty_b < 0.0f) duty_b = 0.0f;
  if (duty_c > 1.0f) duty_c = 1.0f;
  if (duty_c < 0.0f) duty_c = 0.0f;

   // Gán giá tr? CCR (duty cycle) cho 3 pha
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(duty_a * PWM_MAX));  // Phase A
   __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)(duty_b * PWM_MAX));  // Phase B
   __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)(duty_c * PWM_MAX));  // Phase C
}

//---------------------------------------------------------SVPWM
void svpwm (float V_alpha, float V_beta){
	float M_PI = 3.14;
	float pwm_max = 100;
	// Su ly du lieu dau vao
	float v_ref = sqrt(V_alpha*V_alpha + V_beta*V_beta);
	float angle = atan2f(V_beta, V_alpha); 
	// Su ly sector va goc
	float angle_deg = angle * 180.0f / M_PI;  
	if(angle_deg < 0) angle_deg += 360.0f;    
	int sector = (int)(angle_deg / 60.0f) + 1; 
	// Tinh T012
	float T_s = 1.0f / 6666.66f; // 20 kHz
	float Vdc = 24.0f;
		
	float Vmax = Vdc / sqrtf(3.0f);  // Bien do toi da SVPWM
	float Vnorm = v_ref / Vmax;       // Bien do dã chuan hóa (0 - 1)
	float T1 = T_s * Vnorm * sinf((M_PI / 3.0f) - fmodf(angle, M_PI / 3.0f));
	float T2 = T_s * Vnorm * sinf(fmodf(angle, M_PI / 3.0f));
	float T0 = T_s - T1 - T2;  // Zero vector time
	//Tính pwm
	float Ta = (T1 + T2 + T0/2.0f) / T_s;
	float Tb = (T2 + T0/2.0f) / T_s;
	float Tc = (T0/2.0f) / T_s;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Ta * pwm_max);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Tb * pwm_max);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Tc * pwm_max);

}