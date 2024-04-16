#ifndef usonic_h
#define usonic_h

#include "stm32l4xx_hal.h"
#include "avgqueue.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct USonic {
    TIM_HandleTypeDef* trigger_tim;
    TIM_HandleTypeDef* echo_tim;
    AvgQueue last_five_values;
} USonic;

void usonic_ctor(USonic* usonic) {
    if (!usonic) Error_Handler();
    avgqueue_ctor(&usonic->last_five_values);
    for (int i = 0; i < 5; ++i) {
        push(&usonic->last_five_values, 0U);
    }
}

// __HAL_TIM_SET_CAPTUREPOLARITY()
//https://controllerstech.com/input-capture-in-stm32/

#endif
