#ifndef motor_h
#define motor_h

#include "main.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct Motor {
    TIM_HandleTypeDef* pwm_tim;
    GPIO_TypeDef* forward_gpio;
    GPIO_TypeDef* backward_gpio;
    uint32_t pwm_tim_channel;
    uint16_t forward_pin;
    uint16_t backward_pin;
    GPIO_PinState forward_state;
} Motor;

Motor* motor_ctor(TIM_HandleTypeDef *pwm_tim, uint32_t pwm_tim_channel, GPIO_TypeDef *pwm_gpiox, uint16_t pwm_pin, GPIO_TypeDef *forward_gpiox, uint16_t forward_pin, GPIO_TypeDef *backward_gpiox, uint16_t backward_pin) {
    Motor* motor = (Motor*)malloc(sizeof(Motor));
    if (!motor) Error_Handler();
    motor->pwm_tim = pwm_tim;
    motor->forward_gpio = forward_gpiox;
    motor->backward_gpio = backward_gpiox;
    motor->pwm_tim_channel = pwm_tim_channel;
    motor->forward_pin = forward_pin;
    motor->backward_pin = backward_pin;
    motor->forward_state = GPIO_PIN_SET;
    HAL_GPIO_WritePin(forward_gpiox, forward_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(backward_gpiox, backward_pin, GPIO_PIN_RESET);
    return motor;
}

// TODO:
void set_pwm_ccr(Motor* motor, uint32_t ccr_val) {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_tim_channel, ccr_val);
}

void start_pwm(Motor* motor) {
    HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_tim_channel);
}

void stop_pwm(Motor* motor) {
    HAL_TIM_PWM_Stop(motor->pwm_tim, motor->pwm_tim_channel);
}

// TODO
// void set_throttle(Motor* motor) {
// }

void change_direction(Motor* motor) {
    HAL_GPIO_TogglePin(motor->forward_gpio, motor->forward_pin);
    HAL_GPIO_TogglePin(motor->backward_gpio, motor->backward_pin);
}

void set_direction_forward(Motor* motor) {
    HAL_GPIO_WritePin(motor->forward_gpio, motor->forward_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->backward_gpio, motor->backward_pin, GPIO_PIN_RESET);
}

void set_direction_backward(Motor* motor) {
    HAL_GPIO_WritePin(motor->forward_gpio, motor->forward_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->backward_gpio, motor->backward_pin, GPIO_PIN_SET);
}

void motor_dtor(Motor* motor) {
    stop_pwm(motor);
    free(motor);
}




#endif
