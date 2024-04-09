#ifndef servo_h
#define servo_h

#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct Servo {
    TIM_HandleTypeDef* pwm_tim;
    uint32_t ccr;
} Servo;

Servo* servo_ctor() {
    Servo* servo = (Servo*)malloc(sizeof(Servo));
    if (!servo) Error_Handler();
    return servo;
}
void servo_dtor(Servo* servo) {
    free(servo);
}

// __HAL_TIM_SET_CAPTUREPOLARITY()


#endif
