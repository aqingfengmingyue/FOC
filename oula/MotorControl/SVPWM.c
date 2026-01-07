#include <math.h>

// 常数定义
#include "constants.h"
#include "SVPWM.h"

/**
 * SVPWM扇区判断
 * 根据alpha-beta坐标系中的电压矢量判断所属扇区
 *
 * @param valpha α轴电压分量
 * @param vbeta  β轴电压分量
 * @return 扇区号 (1~6)，错误返回-1
 */
int judge_sector(float valpha, float vbeta)
{
    // 预计算√3值

    // 计算三个判断条件
    // A: -(α*√3 + β) > 0
    int A = (-(valpha * SQRT3) - vbeta) > 0.0f ? 1 : 0;

    // B: (α*√3 - β) > 0
    int B = ((valpha * SQRT3) - vbeta) > 0.0f ? 1 : 0;

    // C: β >= 0
    int C = vbeta >= 0.0f ? 1 : 0;

    // 计算扇区编码 (4*A + 2*B + C)
    int sector_code = 4 * A + 2 * B + C;

    // 根据编码确定扇区号
    switch (sector_code)
    {
    case 1:
        return 2;
    case 2:
        return 6;
    case 3:
        return 1;
    case 4:
        return 4;
    case 5:
        return 3;
    case 6:
        return 5; // 注意：MATLAB代码此处返回2，可能是笔误，SVPWM标准扇区应返回5
    default:
        return -1; // 无效扇区
    }
}

/**
 * 限制调制，防止过调制
 * @param Ta 第一个时间分量（输入输出参数）
 * @param Tb 第二个时间分量（输入输出参数）
 * @param arr ARR值（周期）
 */
void limit_modulation(float *Ta, float *Tb, float arr)
{
    float temp = *Ta + *Tb;
    if (temp > arr)
    {
        float scaling_factor = arr / temp;
        *Ta *= scaling_factor;
        *Tb *= scaling_factor;
    }
}

/**
 * 计算SVPWM的CCR值
 * @param sector 扇区号 (1-6)
 * @param valpha α轴电压分量
 * @param vbeta  β轴电压分量
 * @param vdc    直流母线电压
 * @param arr    自动重装载值（PWM周期）
 * @param ccr1   输出：CCR1值
 * @param ccr2   输出：CCR2值
 * @param ccr3   输出：CCR3值
 * @return 成功返回0，失败返回-1
 */
int calculate_ccr(int sector, float valpha, float vbeta, float vdc, uint16_t arr,
                  uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{

    // 输入参数检查
    if (sector < 1 || sector > 6)
    {
        return INVALID_SECTOR;
    }

    // 中间变量
    float T[7] = {0}; // T1-T6，T[0]不使用
    float T7;         // 零矢量时间

    // 基本电压矢量,2/3*Vdc
    float fundamental_vector = 0.6666667f * vdc;

    // 根据扇区计算时间分量
    switch (sector)
    {
    case 1: // 扇区1
        T[6] = (2.0f * vbeta * arr / SQRT3) / fundamental_vector;
        T[4] = ((valpha - vbeta / SQRT3) * arr) / fundamental_vector;
        limit_modulation(&T[4], &T[6], arr);
        T7 = (arr - T[4] - T[6]) / 2.0f;
        *ccr1 = T[4] + T[6] + T7;
        *ccr2 = T[6] + T7;
        *ccr3 = T7;
        break;

    case 2: // 扇区2
        T[2] = ((vbeta / SQRT3 - valpha) * arr) / fundamental_vector;
        T[6] = ((vbeta / SQRT3 + valpha) * arr) / fundamental_vector;
        limit_modulation(&T[2], &T[6], arr);
        T7 = (arr - T[2] - T[6]) / 2.0f;
        *ccr1 = T[6] + T7;
        *ccr2 = T[2] + T[6] + T7;
        *ccr3 = T7;
        break;

    case 3: // 扇区3
        T[2] = SQRT3 * vbeta * arr / vdc;
        T[3] = -(arr / vdc * (1.5f * valpha + SQRT3 / 2.0f * vbeta));
        limit_modulation(&T[2], &T[3], arr);
        T7 = (arr - T[2] - T[3]) / 2.0f;
        *ccr1 = T7;
        *ccr2 = T[2] + T[3] + T7;
        *ccr3 = T[3] + T7;
        break;

    case 4: // 扇区4
        T[1] = (-2.0f * vbeta * arr / SQRT3) / fundamental_vector;
        T[3] = ((-valpha + vbeta / SQRT3) * arr) / fundamental_vector;
        limit_modulation(&T[1], &T[3], arr);
        T7 = (arr - T[1] - T[3]) / 2.0f;
        *ccr1 = T7;
        *ccr2 = T[3] + T7;
        *ccr3 = T[1] + T[3] + T7;
        break;

    case 5: // 扇区5
        T[1] = ((-vbeta / SQRT3 - valpha) * arr) / fundamental_vector;
        T[5] = ((-vbeta / SQRT3 + valpha) * arr) / fundamental_vector;
        limit_modulation(&T[1], &T[5], arr);
        T7 = (arr - T[1] - T[5]) / 2.0f;
        *ccr1 = T[5] + T7;
        *ccr2 = T7;
        *ccr3 = T[1] + T[5] + T7;
        break;

    case 6: // 扇区6
        T[5] = (-2.0f * vbeta * arr / SQRT3) / fundamental_vector;
        T[4] = ((valpha + vbeta / SQRT3) * arr) / fundamental_vector;
        limit_modulation(&T[4], &T[5], arr);
        T7 = (arr - T[4] - T[5]) / 2.0f;
        *ccr1 = T[4] + T[5] + T7;
        *ccr2 = T7;
        *ccr3 = T[5] + T7;
        break;

    default:
        return INVALID_SECTOR;
    }

    // 计算最终的ccr值,ccr值即是PWM波的切换时间
    *ccr1 = (arr - *ccr1);
    *ccr2 = (arr - *ccr2);
    *ccr3 = (arr - *ccr3);

    return 0;
}
