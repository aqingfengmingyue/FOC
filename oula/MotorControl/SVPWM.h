#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

int judge_sector(float valpha, float vbeta);
void limit_modulation(float *Ta, float *Tb, float arr);
int calculate_ccr(int sector, float valpha, float vbeta, float vdc, uint16_t arr, uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3);

#endif