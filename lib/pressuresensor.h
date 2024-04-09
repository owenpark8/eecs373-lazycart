#ifndef pressuresensor_h
#define pressuresensor_h

#include "stm32l4xx_hal.h"

#include <stdlib.h>

typedef struct PressureSensor {
    ADC_HandleTypeDef* hadc;
    uint32_t channel;
    uint32_t raw_value;
    float pressure;
} PressureSensor;

void pressure_sensor_ctor(PressureSensor* ps, ADC_HandleTypeDef* hadc, uint32_t channel) {
    ps->hadc = hadc;
    ps->channel = channel;
}

#endif
