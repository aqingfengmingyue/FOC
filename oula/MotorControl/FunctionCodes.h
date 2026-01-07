// 文件: global_vars.h
#ifndef FUNCTIONCODES_H
#define FUNCTIONCODES_H

#include <stdint.h>
#include "GlobalVars.h"

void clark_transform(float a, float b, float c, float *alpha, float *beta);
void inv_clark_transform(float alpha, float beta, float *a, float *b, float *c);
void park_transform(float alpha, float beta, float theta, float *d, float *q);
void inv_park_transform(float d, float q, float theta, float *alpha, float *beta);
void pid_init(pid_controller *pid, float kp, float ki, float kd, float min, float max);
float pid_calculate(pid_controller *pid, float ref, float fed, float dt);

#endif