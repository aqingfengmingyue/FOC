#ifndef LBG_H
#define LBG_H

void Luenberger_Update(LuenbergerObserver *lbg,
                       MotorParam *motor,
                       float u_alpha, // 电压
                       float i_alpha  // 实际电流
);

#endif
