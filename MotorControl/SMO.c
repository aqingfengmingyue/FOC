#include "GlobalVars.h"
#include "SMO.h"
#include "constants.h"
#include <math.h>

/**
 * @brief 滑模观测器初始化
 */
void SMO_Init(SMO_HandleTypeDef *smo, float Rs, float Ld, float Lq, float Ts, float Kslide)
{
    smo->Rs = Rs;
    smo->Ld = Ld;
    smo->Lq = Lq;
    smo->Ts = Ts;
    smo->Kslide = Kslide;

    smo->i_alpha_est = 0.0f;
    smo->i_beta_est = 0.0f;
    smo->z_alpha = 0.0f;
    smo->z_beta = 0.0f;
}

/**
 * @brief 滑模观测器运行一次
 * @param smo: 观测器句柄
 * @param u_alpha: α轴电压
 * @param u_beta: β轴电压
 * @param i_alpha: α轴电流测量值
 * @param i_beta: β轴电流测量值
 * @param omega_e: 电角速度(来自PLL反馈)
 */
void SMO_Run(SMO_HandleTypeDef *smo,
             float u_alpha, float u_beta,
             float i_alpha, float i_beta,
             float omega_e)
{

    // 1. 计算电流误差 (估算值 - 测量值)
    float err_alpha = smo->i_alpha_est - i_alpha;
    float err_beta = smo->i_beta_est - i_beta;

    // 2. 滑模控制律: 通常使用sign函数 + 饱和函数
    // 为了减少抖振，可以使用饱和函数代替纯sign
    float sat_alpha, sat_beta;
    float boundary = 0.1f; // 边界层厚度，需要根据实际调试

    if (err_alpha > boundary)
    {
        sat_alpha = 1.0f;
    }
    else if (err_alpha < -boundary)
    {
        sat_alpha = -1.0f;
    }
    else
    {
        sat_alpha = err_alpha / boundary;
    }

    if (err_beta > boundary)
    {
        sat_beta = 1.0f;
    }
    else if (err_beta < -boundary)
    {
        sat_beta = -1.0f;
    }
    else
    {
        sat_beta = err_beta / boundary;
    }

    // 滑模控制信号 (z = Kslide * sat(err))
    smo->z_alpha = smo->Kslide * sat_alpha;
    smo->z_beta = smo->Kslide * sat_beta;

    // 3. 电流观测器离散方程 (你的方程38的α轴实现)
    // 方程: i_hat[k+1] = i_hat[k] + Ts*[-Rs/Ld * i_hat[k] - ωe*(Ld-Lq)/Ld * i_hat[k] + 1/Ld * u[k] - 1/Ld * v[k]]
    // 注意: 这里的v就是滑模控制信号z

    // α轴观测器
    float di_alpha = (-smo->Rs / smo->Ld) * smo->i_alpha_est - (omega_e * (smo->Ld - smo->Lq) / smo->Ld) * smo->i_beta_est // 交叉耦合项
                     + (1.0f / smo->Ld) * u_alpha - (1.0f / smo->Ld) * smo->z_alpha;

    smo->i_alpha_est += smo->Ts * di_alpha;

    // β轴观测器 (类似，注意符号)
    float di_beta = (-smo->Rs / smo->Ld) * smo->i_beta_est + (omega_e * (smo->Ld - smo->Lq) / smo->Ld) * smo->i_alpha_est // 交叉耦合项
                    + (1.0f / smo->Ld) * u_beta - (1.0f / smo->Ld) * smo->z_beta;

    smo->i_beta_est += smo->Ts * di_beta;
}

// 初始化LPF
void LPF_Init(LPF_HandleTypeDef *lpf, float fc, float Ts)
{
    float temp = TWO_PI * fc * Ts;
    lpf->alpha = temp / (1.0f + temp);
    lpf->y_prev = 0.0f;
}

/**
 * @brief 低通滤波器运行一次
 * @param lpf: 滤波器句柄
 * @param u: 送入滤波器的输入
 * @param alpha: 0-1之间的一个系数，决定滤波器的截止频率，通常根据采样时间和期望的截止频率计算得到
 * @param y_prev: 上一时刻滤波器的输出
 * @return y: 经过滤波器的输出
 */

// LPF运行
float LPF_Run(LPF_HandleTypeDef *lpf, float u)
{
    float y = lpf->alpha * u + (1.0f - lpf->alpha) * lpf->y_prev;
    lpf->y_prev = y;
    return y;
}

// PLL初始化
void PLL_Init(PLL_HandleTypeDef *pll, float Kp, float Ki, float Ts)
{
    pll->Kp = Kp;
    pll->Ki = Ki;
    pll->Ts = Ts;
    pll->integral = 0.0f;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->err_prev = 0.0f;
}

/**
 * @brief PLL运行
 * @param e_alpha: 滤波后的α轴反电动势
 * @param e_beta: 滤波后的β轴反电动势
 * @retval 估算的电角度
 */
float PLL_Run(PLL_HandleTypeDef *pll, float e_alpha, float e_beta)
{
    // 1. 归一化反电动势(消除幅值影响)
    float E_mag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);

    // 避免除以零
    if (E_mag < 0.001f)
    {
        E_mag = 0.001f;
    }

    float e_alpha_norm = e_alpha / E_mag;
    float e_beta_norm = e_beta / E_mag;

    // 2. 鉴相器: 计算误差 sin(θ_true - θ_est)
    // e_norm = [ -sinθ; cosθ ]
    float sin_theta = sinf(pll->theta_est);
    float cos_theta = cosf(pll->theta_est);

    // 误差 = 叉积 = e_α_norm * cosθ + e_β_norm * sinθ? 注意符号!
    // 标准形式: err = -e_α * cosθ - e_β * sinθ
    float err = -e_alpha_norm * cos_theta - e_beta_norm * sin_theta;

    // 可选: 对误差限幅
    if (err > 1.0f)
        err = 1.0f;
    if (err < -1.0f)
        err = -1.0f;

    // 3. PI控制器 (后向欧拉积分)
    pll->integral += pll->Ki * err * pll->Ts;

    // 抗积分饱和
    float max_omega = 20.0f; // 最大速度限制 rad/s
    if (pll->integral > max_omega)
        pll->integral = max_omega;
    if (pll->integral < -max_omega)
        pll->integral = -max_omega;

    // PI输出 = 电角速度
    pll->omega_est = pll->Kp * err + pll->integral;

    // 4. VCO: 速度积分得到角度 (前向欧拉)
    pll->theta_est += pll->omega_est * pll->Ts;

    // 角度归一化到 [0, 2pi]
    if (pll->theta_est > TWO_PI)
    {
        pll->theta_est -= TWO_PI;
    }
    else if (pll->theta_est < 0.0f)
    {
        pll->theta_est += TWO_PI;
    }

    return pll->theta_est;
}