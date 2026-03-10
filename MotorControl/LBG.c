#include "LBG.h"
#include "GlobalVars.h"
void Luenberger_Update(
    LuenbergerObserver *lbg,
    MotorParam *motor,
    float u_alpha, // 电压
    float i_alpha  // 实际电流
)
{
    float i_hat = lbg->i_hat;
    float E_hat = lbg->E_hat;

    float T = motor->T;
    float R = motor->R;
    float L = motor->L;
    float l1 = motor->l1;
    float l2 = motor->l2;

    float error = i_hat - i_alpha;

    float i_hat_next =
        i_hat - T * (R / L) * i_hat - T * (1.0f / L) * E_hat + T * (1.0f / L) * u_alpha + l1 * T * error;

    float E_hat_next =
        E_hat + l2 * T * error;

    lbg->i_hat = i_hat_next;
    lbg->E_hat = E_hat_next;
}