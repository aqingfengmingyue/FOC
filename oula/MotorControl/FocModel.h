#ifndef FOCMODEL_H
#define FOCMODEL_H
void foc_control(void);
void foc_open_loop(int16_t ref_speed);
void align_rotor_to_zero(void);
void update_pwm_duty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
void motor_state_reset(motor_state *motor, float ref_speed);
void foc_curloop(void);

#endif