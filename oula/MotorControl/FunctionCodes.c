#include <math.h>
#include "FunctionCodes.h"
#include "GlobalVars.h"
#include "constants.h"

/**
 * Clark变换（等幅值变换）
 * 将三相静止坐标系(a,b,c)转换为两相静止坐标系(alpha,beta)
 * 变换系数：2/3
 */
void clark_transform(float a, float b, float c, float *alpha, float *beta)
{
    /* 通用Clark变换公式（适用于任何三相系统）*/

    // 计算alpha分量：α = (2/3) * [a - 0.5*b - 0.5*c]
    *alpha = 0.6666667f * (a - 0.5f * b - 0.5f * c);

    // 计算beta分量：β = (2/3) * (√3/2) * (b - c) = (1/√3) * (b - c)
    // 使用预计算的1/√3值，避免重复计算平方根
    *beta = INV_SQRT3 * (b - c);
}

/**
 * 克拉克逆变换（反Clark变换）
 * 将两相静止坐标系(alpha,beta)转换为三相静止坐标系(a,b,c)
 * 采用等幅值变换系数
 * 注意：输出三相中，第三相c可通过c = -a - b得到（当系统平衡时）
 */
void inv_clark_transform(float alpha, float beta, float *a, float *b, float *c)
{
    /*
     * 反Clark变换矩阵（等幅值变换）：
     * | a |   | 1       0      |   | alpha |
     * | b | = | -1/2   √3/2    | * | beta  |
     * | c |   | -1/2  -√3/2    |
     *
     * 注意：这是完整的3相输出，不假设系统平衡
     */

    // 计算a相：a = alpha
    *a = alpha;

    // 计算b相：b = -0.5*alpha + (√3/2)*beta
    *b = -0.5f * alpha + SQRT3_BY_2 * beta;

    // 计算c相：c = -0.5*alpha - (√3/2)*beta
    *c = -0.5f * alpha - SQRT3_BY_2 * beta;
}

/**
 * Park变换（等幅值）
 * 将两相静止坐标系(alpha,beta)转换为两相同步旋转坐标系(d,q)
 *
 * @param alpha 静止坐标系α轴分量
 * @param beta  静止坐标系β轴分量
 * @param theta 旋转角度（弧度）
 * @param d     输出：同步旋转坐标系d轴分量（直轴）
 * @param q     输出：同步旋转坐标系q轴分量（交轴）
 *
 * 变换矩阵：
 * | d |   | cosθ   sinθ |   | α |
 * | q | = |-sinθ   cosθ | * | β |
 *
 * 注意：常见的FOC中，d轴对应励磁分量，q轴对应转矩分量
 */
void park_transform(float alpha, float beta, float theta, float *d, float *q)
{
    // 预计算三角函数值
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    // Park变换公式
    // d = α*cosθ + β*sinθ
    // q = -α*sinθ + β*cosθ
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

/**
 * Park逆变换（反Park变换）
 * 将同步旋转坐标系(d,q)转换回两相静止坐标系(alpha,beta)
 *
 * 逆变换矩阵：
 * | α |   | cosθ   -sinθ |   | d |
 * | β | = | sinθ    cosθ | * | q |
 */
void inv_park_transform(float d, float q, float theta, float *alpha, float *beta)
{
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    // 反Park变换公式
    // α = d*cosθ - q*sinθ
    // β = d*sinθ + q*cosθ
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

/**
 * PID控制器结构体
 * 采用位置式PID算法
 */

/**
 * 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 * @param min 输出最小值
 * @param max 输出最大值
 */
void pid_init(pid_controller *pid, float kp, float ki, float kd, float min, float max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->output_min = min;
    pid->output_max = max;

    // 积分限幅通常设置为输出限幅的50%-100%
    pid->integral_max = max * 0.5f;
    pid->integral_min = min * 0.5f;
}
/**
 * 计算PID控制器输出
 * @param pid PID控制器指针
 * @param ref 参考值
 * @param fed 反馈值
 * @param dt  时间间隔（秒）
 * @return    控制器输出
 */
float pid_calculate(pid_controller *pid, float ref, float fed, float dt)
{
    float error = ref - fed;
    float p_term = pid->kp * error;

    // 条件积分：只有当控制器未饱和或与误差方向一致时才积分
    float tentative_i = pid->integral + error * dt;

    // 积分限幅
    if (tentative_i > pid->integral_max)
        tentative_i = pid->integral_max;
    else if (tentative_i < pid->integral_min)
        tentative_i = pid->integral_min;

    // 计算临时 i_term，用于判断输出是否饱和（可选）
    float i_term = pid->ki * tentative_i;
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    float output = p_term + i_term + d_term;

    // 输出限幅
    if (output > pid->output_max)
    {
        // 如果输出被饱和且积分会使饱和更严重，则不更新积分（条件积分）
        // 这里简单不更新积分以防windup
        // pid->integral 保持原值
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }
    else
    {
        // 输出未饱和，确认更新积分
        pid->integral = tentative_i;
    }

    pid->prev_error = error;
    pid->output = output;
    return output;
}
