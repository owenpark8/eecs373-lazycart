#ifndef servo_h
#define servo_h

#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

#define ZERO_DEG 29
#define NINETY_DEG 74
#define ONE_EIGHTY_DEG 119

typedef struct Servo {
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_tim_channel;
    uint32_t ccr;
} Servo;

void servo_ctor(Servo* servo, TIM_HandleTypeDef* pwm_tim, uint32_t pwm_tim_channel) {
    if (!servo) Error_Handler();
    servo->pwm_tim = pwm_tim;
    servo->pwm_tim_channel = pwm_tim_channel;
}

void servo_start_pwm(Servo* servo) {
    HAL_TIM_PWM_Stop(servo->pwm_tim, servo->pwm_tim_channel);
    HAL_TIM_PWM_Start(servo->pwm_tim, servo->pwm_tim_channel);
}

void servo_stop_pwm(Servo* servo) {
    HAL_TIM_PWM_Stop(servo->pwm_tim, servo->pwm_tim_channel);
}

void servo_set_pwm_ccr(Servo* servo, uint32_t ccr_val) {
    __HAL_TIM_SET_COMPARE(servo->pwm_tim, servo->pwm_tim_channel, ccr_val);
}

void servo_set_zero_deg(Servo* servo) {
    servo_set_pwm_ccr(servo, ZERO_DEG);
}

void servo_set_ninety_deg(Servo* servo) {
    servo_set_pwm_ccr(servo, NINETY_DEG);
}

void servo_set_one_eighty_deg(Servo* servo) {
    servo_set_pwm_ccr(servo, ONE_EIGHTY_DEG);
}


#endif
