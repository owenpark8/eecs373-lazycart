#ifndef usonic_h
#define usonic_h

#include "main.h"
#include "avgqueue.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct USonic {
    TIM_HandleTypeDef* trigger_tim;
    TIM_HandleTypeDef* echo_tim;
    AvgQueue* last_five_values;
} USonic;

USonic* usonic_ctor() {
    USonic* usonic = (USonic*)malloc(sizeof(USonic));
    if (!usonic) Error_Handler();
    init(usonic->last_five_values);
    for (int i = 0; i < 5; ++i) {
        push(usonic->last_five_values, 0U);
    }
    return usonic;
}
void usonic_dtor(USonic* usonic) {
    free(usonic);
}

// __HAL_TIM_SET_CAPTUREPOLARITY()
//https://controllerstech.com/input-capture-in-stm32/

#endif
