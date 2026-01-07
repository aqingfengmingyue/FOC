#ifndef GLOBALVARS_H
#define GLOBALVARS_H

#include <stdint.h>

// 结构体定义
typedef struct
{
    float ref_speed;
    float fed_speed;
    float theta;
    float ia;
    float ib;
    float ic;
    uint8_t status;
    int changed_status;
    uint8_t flag_CurrLoop;
    int8_t spin_flag;
} motor_state;

typedef struct
{
    float ref_id;
    float fed_id;
    float ref_iq;
    float fed_iq;
    float alpha;
    float beta;
    float genera_vq;
    float genera_vd;
    int8_t change_PIControl;
} current_loop_params;

typedef struct
{
    int adc_offset; // 判断adc是否校准完成
    int cnt;
    uint16_t adc_first;
    uint16_t adc_second;
    uint16_t adc_third;
    uint16_t ia_offset;
    uint16_t ib_offset;
    uint16_t ic_offset;

} adc_params;

typedef struct
{
    float valpha;
    float vbeta;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;
    float vdc;
    uint16_t arr;
    int sector;

} svpwm_params;

typedef struct
{
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数
    float dt; // 采样时间间隔（秒）

    float output; // 控制器输出

    float integral;   // 积分项累计值
    float prev_error; // 上一次误差（用于微分计算）

    float output_max; // 输出上限
    float output_min; // 输出下限

    float integral_max; // 积分项上限（抗积分饱和）
    float integral_min; // 积分项下限
} pid_controller;

typedef struct
{
    pid_controller speed_pid;
    pid_controller cur_pid_q;
    pid_controller cur_pid_d;

    float max_speed;
    float max_current;
} motor_controller;

typedef struct
{
    float curr_HallTheta;
    float last_HallTheta;
    float hall_theta_add;
    float hall_speed;
    uint8_t in_HallCnt;
    uint32_t curr_time;
    uint32_t last_time;
    uint32_t delta_count;
    uint32_t record_DeltaCount[6];
    uint8_t speed_update;
    uint32_t arr;
    uint8_t curr_HallValue;
    uint8_t last_HallValue;

} hall_sensor_params;

// 在GlobalVars.h中添加以下结构体
typedef struct
{
    uint8_t enable;         // 开环使能标志
    float speed;            // 开环目标速度 (rad/s, 电角度速度)
    float angle;            // 开环角度 (电角度, rad)
    float vd;               // 开环d轴电压
    float vq;               // 开环q轴电压
    float start_angle;      // 开环起始对齐角度
    uint32_t start_time;    // 开环启动时间
    uint8_t alignment_done; // 转子对齐完成标志
} OpenLoopControl_t;

// 全局声明
extern OpenLoopControl_t motor1_openloop;
extern motor_state motor1_state;
extern motor_controller motor1_control;
extern hall_sensor_params hall1_params;
extern svpwm_params svpwm1_params;
extern adc_params adc_data;
extern current_loop_params motor1_clp;
extern float load_data[14];

#endif