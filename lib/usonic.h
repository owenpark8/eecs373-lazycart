#ifndef usonic_h
#define usonic_h

#include "stm32l4xx_hal.h"
#include "avgqueue.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct USonic {
    TIM_HandleTypeDef* trigger_tim;
    TIM_HandleTypeDef* echo_tim;
    uint32_t trigger_tim_channel;
    uint32_t echo_tim_channel;
    AvgQueue last_five_values;
    uint32_t pulse_width;
    uint8_t capture_state; // 0 for rising edge, 1 for falling edge
} USonic;

void usonic_ctor(USonic* usonic, TIM_HandleTypeDef* trigger_tim, uint32_t trigger_tim_channel, TIM_HandleTypeDef* echo_tim, uint32_t echo_tim_channel) {
    if (!usonic) Error_Handler();
    usonic->trigger_tim = trigger_tim;
    usonic->echo_tim = echo_tim;
    usonic->trigger_tim_channel = trigger_tim_channel;
    usonic->echo_tim_channel = echo_tim_channel;
    usonic->capture_state = 0;
    usonic->pulse_width = 0;
    // avgqueue_ctor(&usonic->last_five_values);
    // for (int i = 0; i < 5; ++i) {
    //     push(&usonic->last_five_values, 0U);
    // }
}

void usonic_start_pwm(USonic* usonic) {
    HAL_TIM_PWM_Stop(usonic->trigger_tim, usonic->trigger_tim_channel);
    HAL_TIM_PWM_Start(usonic->trigger_tim, usonic->trigger_tim_channel);
}

void usonic_stop_pwm(USonic* usonic) {
    HAL_TIM_PWM_Stop(usonic->trigger_tim, usonic->trigger_tim_channel);
}

void usonic_start_capture(USonic* usonic) {
    HAL_TIM_IC_Start_IT(usonic->echo_tim, usonic->echo_tim_channel);
}

void usonic_echo_callback(USonic* usonic) {
    if (usonic->capture_state == 0) {
        __HAL_TIM_SET_CAPTUREPOLARITY(usonic->echo_tim, usonic->echo_tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
        __HAL_TIM_SET_COUNTER(usonic->echo_tim, 0);
        usonic->capture_state = 1;
    } else {
        __HAL_TIM_SET_CAPTUREPOLARITY(usonic->echo_tim, usonic->echo_tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
        usonic->pulse_width = __HAL_TIM_GET_COUNTER(usonic->echo_tim);
        usonic->capture_state = 0;
    }
    // push(&usonic->last_five_values, pulse_width);
    // pop(&usonic->last_five_values);
}

uint32_t usonic_get_distance(USonic* usonic) {
    return usonic->pulse_width;
    // return get_average(&usonic->last_five_values);
}


// __HAL_TIM_SET_CAPTUREPOLARITY()
//https://controllerstech.com/input-capture-in-stm32/

#endif
