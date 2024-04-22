#ifndef ir_h
#define ir_h

#include "stdlib.h"

#include "stm32l4xx_hal.h"

typedef struct IR {
    ADC_HandleTypeDef* hadc;
    uint16_t adc_val;
} IR;

void ir_ctor(IR* ir, ADC_HandleTypeDef* hadc) {
    if (!ir) Error_Handler();
    ir->hadc = hadc;
}

void ir_start_read(IR* ir) {
	HAL_ADC_Start_IT(ir->hadc);
}

void ir_stop_read(IR* ir) {
	HAL_ADC_Stop_IT(ir->hadc);
}

void ir_adc_callback(IR* ir) {
	ir->adc_val = HAL_ADC_GetValue(ir->hadc);
}

static inline const uint16_t ir_get_value(IR* ir) {
    return ir->adc_val;
}


#endif
