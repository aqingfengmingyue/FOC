#include "GlobalVars.h"

// 定义所有全局变量（集中管理）
motor_state motor1_state = {
    .ia = 0.0f,
    .ib = 0.0f,
    .ic = 0.0f,
    .ref_speed = 0.0f,
    .fed_speed = 0.0f,
    .theta = 0.0f,
    .status = 0,
    .changed_status = 0,
    .flag_CurrLoop = 0,
    .spin_flag = 0

};

current_loop_params motor1_clp = {
    .ref_id = 0.0f,
    .fed_id = 0.0f,
    .ref_iq = 0.0f,
    .fed_iq = 0.0f,
    .alpha = 0.0f,
    .beta = 0.0f,
    .genera_vq = 0.0f,
    .genera_vd = 0.0f,
    .change_PIControl = 0};

svpwm_params svpwm1_params = {
    .valpha = 0.0f,
    .vbeta = 0.0f,
    .ccr1 = 0,
    .ccr2 = 0,
    .ccr3 = 0,
    .vdc = 24.0f,
    .arr = 8000 - 1,
    .sector = 0};

motor_controller motor1_control = {
    .speed_pid = {
        .kp = 0.0001185f,
        .ki = 0.01415f,
        .kd = 0.0f,
        .dt = 0.001f,
        .output = 0.0f,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .output_max = 3.0f,   // 需要设置合理的上限,速度环输出的是参考电流
        .output_min = -3.0f,  // 需要设置合理的下限
        .integral_max = 2.7f, // 积分限幅(理论值2083.33*0.6=1250)
        .integral_min = -2.7f},
    .cur_pid_q = {.kp = 5.35f, .ki = 6000.0f,
                  .kd = 0.0f, // 电流环通常不需要微分
                  .dt = 0.0001f,
                  .output = 0.0f,
                  .integral = 0.0f,
                  .prev_error = 0.0f,
                  .output_max = 12.471f, // 电流限制
                  .output_min = -12.471f,
                  .integral_max = 1.1471f, // 电流积分限制:这个积分限幅的最大值应为输出最大值的0.9倍左右
                  .integral_min = -1.1471f},
    .cur_pid_d = {.kp = 0.26f, .ki = 300.0f,
                  .kd = 0.0f, // 电流环通常不需要微分
                  .dt = 0.0001f,
                  .output = 0.0f,
                  .integral = 0.0f,
                  .prev_error = 0.0f,
                  .output_max = 12.471f, // 电流限制，电流环输出的是参考电压
                  .output_min = -12.471f,
                  .integral_max = 1.1471f, // 电流积分限制:这个积分限幅的最大值应为输出最大值的0.9倍左右
                  .integral_min = -1.1471f},
    .max_speed = 1200.0f,
    .max_current = 3.0f};

hall_sensor_params hall1_params = {
    .curr_HallTheta = 0.0f,
    .last_HallTheta = 0.0f,
    .hall_theta_add = 0.0f,
    .hall_speed = 0.0f,
    .in_HallCnt = 0,
    .curr_time = 0,
    .last_time = 0,
    .delta_count = 1,
    .record_DeltaCount = {0},
    .speed_update = 0,
    .arr = 65535,
    .curr_HallValue = 0x00,
    .last_HallValue = 0x00};

adc_params adc_data = {
    .adc_offset = 0,
    .cnt = 0,
    .adc_first = 0,
    .adc_second = 0,
    .adc_third = 0,
    .ia_offset = 0,
    .ib_offset = 0,
    .ic_offset = 0};

// 在GlobalVars.c中定义和初始化
OpenLoopControl_t motor1_openloop = {
    .enable = 1,     // 默认使能开环
    .speed = 100.0f, // 约95.5RPM (对于4极电机: 10/(2π*4)*60 ≈ 95.5)
    .angle = 0.0f,
    .vd = 0.0f, // d轴电压，通常为0
    .vq = 2.0f, // q轴电压，从小开始（2V）
    .start_angle = 0.0f,
    .start_time = 0,
    .alignment_done = 0};

float load_data[14] = {0};