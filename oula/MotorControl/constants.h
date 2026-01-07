#ifndef CONSTANTS_H
#define CONSTANTS_H

// 在头文件中声明为extern
extern const float SQRT3;
extern const float INV_SQRT3;
extern const int INVALID_SECTOR;
extern const float SQRT3_BY_2; // √3/2
extern const float PI;
extern const float PI_BY_3;
extern const float TWO_PI;
extern const int POLE_PAIRS;
extern float PHASE_SHIFT_ANGLE;
// 电机特性常数
typedef struct
{
    const float pole_pairs;        // 极对数
    const float torque_constant;   // 转矩常数 (Nm/A)
    const float back_emf_constant; // 反电势常数 (V/krpm)
    const float stator_resistance; // 定子电阻 (Ω)
    const float stator_inductance; // 定子电感 (H)
    const float inertia;           // 转动惯量 (kg·m²)
} motor_parameters;

// 预定义的电机参数
extern const motor_parameters motor1;

#endif