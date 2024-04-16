#ifndef drive_h
#define drive_h

#include "motor.h"

#define SPIN_SPEED 600
#define DRIVE_SPEED 500

void spin_left(Motor* left, Motor* right) {
    set_pwm_ccr(left, SPIN_SPEED);
    set_pwm_ccr(right, SPIN_SPEED);
    set_direction_forward(right);
    set_direction_backward(left);
}

void spin_right(Motor* left, Motor* right) {
    set_pwm_ccr(left, SPIN_SPEED);
    set_pwm_ccr(right, SPIN_SPEED);
    set_direction_forward(left);
    set_direction_backward(right);
}

void drive_forward(Motor* left, Motor* right) {
    set_pwm_ccr(left, 500);
    set_pwm_ccr(right, 500);
    set_direction_forward(left);
    set_direction_forward(right);
}

void stop_drive(Motor* left, Motor* right) {
    set_pwm_ccr(left, 0U);
    set_pwm_ccr(right, 0U);
}

#endif
