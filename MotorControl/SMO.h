#ifndef SMO_H
#define SMO_H

void SMO_Init(SMO_HandleTypeDef *smo, float Rs, float Ld, float Lq, float Ts, float Kslide);
void SMO_Run(SMO_HandleTypeDef *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float omega_e);
void LPF_Init(LPF_HandleTypeDef *lpf, float fc, float Ts);
float LPF_Run(LPF_HandleTypeDef *lpf, float u);
void PLL_Init(PLL_HandleTypeDef *pll, float Kp, float Ki, float Ts);
float PLL_Run(PLL_HandleTypeDef *pll, float e_alpha, float e_beta);

#endif
