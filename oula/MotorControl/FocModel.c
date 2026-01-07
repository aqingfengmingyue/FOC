#include "GlobalVars.h"
#include "FunctionCodes.h"
#include "constants.h"
#include "SVPWM.h"
#include "FocModel.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

// 在您的代码中（比如SVPWM.c或main.c）添加这个函数
void update_pwm_duty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    // 方法1：直接操作寄存器（最快）
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;
}

static inline float constrain_local(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void foc_control(void)
{

    clark_transform(motor1_state.ia, motor1_state.ib, motor1_state.ic, &motor1_clp.alpha, &motor1_clp.beta);       // Clark变换
    park_transform(motor1_clp.alpha, motor1_clp.beta, motor1_state.theta, &motor1_clp.fed_id, &motor1_clp.fed_iq); // Park变换
    float vq, vd;

    // 电流环控制（PID 输出做限幅以防过驱动）
    if (motor1_clp.change_PIControl == 0)
    {
        vq = pid_calculate(&motor1_control.cur_pid_q, motor1_clp.ref_iq, motor1_clp.fed_iq, motor1_control.cur_pid_q.dt);
        vd = pid_calculate(&motor1_control.cur_pid_d, motor1_clp.ref_id, motor1_clp.fed_id, motor1_control.cur_pid_d.dt);
    }
    if (vq >= motor1_clp.genera_vq || motor1_clp.change_PIControl)
    {
        motor1_clp.genera_vq = pid_calculate(&motor1_control.cur_pid_q, motor1_clp.ref_iq, motor1_clp.fed_iq, motor1_control.cur_pid_q.dt);
        motor1_clp.genera_vd = pid_calculate(&motor1_control.cur_pid_d, motor1_clp.ref_id, motor1_clp.fed_id, motor1_control.cur_pid_d.dt);
        motor1_clp.change_PIControl = 1;
    }
    inv_park_transform(motor1_clp.genera_vd, motor1_clp.genera_vq, motor1_state.theta, &svpwm1_params.valpha, &svpwm1_params.vbeta); // 逆Park变换
    svpwm1_params.sector = judge_sector(svpwm1_params.valpha, svpwm1_params.vbeta);                                                  // 判断扇区
    calculate_ccr(svpwm1_params.sector, svpwm1_params.valpha, svpwm1_params.vbeta, svpwm1_params.vdc, svpwm1_params.arr,
                  &svpwm1_params.ccr1, &svpwm1_params.ccr2, &svpwm1_params.ccr3); // 计算CCR值

    // CCR 饱和保护，确保在 [0, arr] 范围内
    if (svpwm1_params.ccr1 < 0)
        svpwm1_params.ccr1 = 0;
    if (svpwm1_params.ccr2 < 0)
        svpwm1_params.ccr2 = 0;
    if (svpwm1_params.ccr3 < 0)
        svpwm1_params.ccr3 = 0;
    if (svpwm1_params.ccr1 > svpwm1_params.arr)
        svpwm1_params.ccr1 = svpwm1_params.arr;
    if (svpwm1_params.ccr2 > svpwm1_params.arr)
        svpwm1_params.ccr2 = svpwm1_params.arr;
    if (svpwm1_params.ccr3 > svpwm1_params.arr)
        svpwm1_params.ccr3 = svpwm1_params.arr;
}

// 转子对齐函数（关键！）
void align_rotor_to_zero(void)
{

    // 1. 施加d轴电流锁定转子（不施加q轴电流）
    float vd = 0.0f;          // d轴电压，锁定磁场
    float vq = 0.0f;          // q轴电压，对齐时设为0
    float align_angle = 0.0f; // 目标对齐角度（电角度）

    // 固定角度为0，施加d轴电压
    float align_voltage = 5.0f; // 对齐电压
    vd = align_voltage;         // d轴电压，将转子拉到0电角度位置
    vq = 0.0f;                  // q轴电压为0，不产生转矩

    // 计算αβ电压
    float valpha, vbeta;
    inv_park_transform(vd, vq, align_angle, &valpha, &vbeta);

    // 生成SVPWM
    uint8_t sector = judge_sector(valpha, vbeta);
    calculate_ccr(sector, valpha, vbeta, svpwm1_params.vdc, svpwm1_params.arr,
                  &svpwm1_params.ccr1, &svpwm1_params.ccr2, &svpwm1_params.ccr3);

    // 立即更新PWM输出
    update_pwm_duty(svpwm1_params.ccr1, svpwm1_params.ccr2, svpwm1_params.ccr3);
    HAL_Delay(100);
}

// 最简化的开环测试，验证基本功能
void foc_open_loop(int16_t ref_speed)
{
    const float fe = ref_speed / 15.0f;
    const float delta_theta = 0.0006283f * fe; // 理论值为：2pi*fe*Ts,Ts为foc算法的执行频率

    // 设置角度增量
    motor1_state.theta += delta_theta;

    // 角度归一化
    if (motor1_state.theta > 6.283185f)
        motor1_state.theta -= 6.283185f;
    else if (motor1_state.theta < 0.0f)
    {
        motor1_state.theta += 6.283185f;
    }

    // 2. 设置dq电压（从很小开始）
    const float vd = 0.0f; // d轴：通常为0
    motor1_clp.genera_vq = 0.158f * fe;
    // 3. 逆Park变换（关键！）
    float valpha, vbeta;
    inv_park_transform(vd, motor1_clp.genera_vq, motor1_state.theta, &valpha, &vbeta);

    // 5. 生成PWM
    svpwm1_params.sector = judge_sector(valpha, vbeta);
    calculate_ccr(svpwm1_params.sector, valpha, vbeta, svpwm1_params.vdc, svpwm1_params.arr,
                  &svpwm1_params.ccr1, &svpwm1_params.ccr2, &svpwm1_params.ccr3);
}

void motor_state_reset(motor_state *motor, float ref_speed)
{
    motor->ref_speed = ref_speed;
    motor->fed_speed = 0.0f;
    motor->theta = 0.0f;
    motor->ia = 0.0f;
    motor->ib = 0.0f;
    motor->ic = 0.0f;
    motor->status = 0;
}

// foc电流环控制

void foc_curloop(void)

{
    motor1_clp.ref_id = -0.009f;
    motor1_clp.ref_iq = 0.04f; // 电流环测试，参考iq为1.0

    clark_transform(motor1_state.ia, motor1_state.ib, motor1_state.ic, &motor1_clp.alpha, &motor1_clp.beta);       // Clark变换
    park_transform(motor1_clp.alpha, motor1_clp.beta, motor1_state.theta, &motor1_clp.fed_id, &motor1_clp.fed_iq); // Park变换
    float vq, vd;

    // 电流环控制（PID 输出做限幅以防过驱动）
    if (motor1_clp.change_PIControl == 0)
    {
        vq = pid_calculate(&motor1_control.cur_pid_q, motor1_clp.ref_iq, motor1_clp.fed_iq, motor1_control.cur_pid_q.dt);
        vd = pid_calculate(&motor1_control.cur_pid_d, motor1_clp.ref_id, motor1_clp.fed_id, motor1_control.cur_pid_d.dt);
    }
    if (vq >= motor1_clp.genera_vq || motor1_clp.change_PIControl)
    {
        motor1_clp.genera_vq = pid_calculate(&motor1_control.cur_pid_q, motor1_clp.ref_iq, motor1_clp.fed_iq, motor1_control.cur_pid_q.dt);
        motor1_clp.genera_vd = pid_calculate(&motor1_control.cur_pid_d, motor1_clp.ref_id, motor1_clp.fed_id, motor1_control.cur_pid_d.dt);
        motor1_clp.change_PIControl = 1;
    }
    inv_park_transform(motor1_clp.genera_vd, motor1_clp.genera_vq, motor1_state.theta, &svpwm1_params.valpha, &svpwm1_params.vbeta); // 逆Park变换
    svpwm1_params.sector = judge_sector(svpwm1_params.valpha, svpwm1_params.vbeta);                                                  // 判断扇区
    calculate_ccr(svpwm1_params.sector, svpwm1_params.valpha, svpwm1_params.vbeta, svpwm1_params.vdc, svpwm1_params.arr,
                  &svpwm1_params.ccr1, &svpwm1_params.ccr2, &svpwm1_params.ccr3); // 计算CCR值

    // CCR 饱和保护，确保在 [0, arr] 范围内
    if (svpwm1_params.ccr1 < 0)
        svpwm1_params.ccr1 = 0;
    if (svpwm1_params.ccr2 < 0)
        svpwm1_params.ccr2 = 0;
    if (svpwm1_params.ccr3 < 0)
        svpwm1_params.ccr3 = 0;
    if (svpwm1_params.ccr1 > svpwm1_params.arr)
        svpwm1_params.ccr1 = svpwm1_params.arr;
    if (svpwm1_params.ccr2 > svpwm1_params.arr)
        svpwm1_params.ccr2 = svpwm1_params.arr;
    if (svpwm1_params.ccr3 > svpwm1_params.arr)
        svpwm1_params.ccr3 = svpwm1_params.arr;
}
