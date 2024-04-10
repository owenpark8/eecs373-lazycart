#ifndef weightdisplay_h
#define weightdisplay_h

#include "stm32l4xx_hal.h"

#include "ILI9225.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define WEIGHT_CHAR_BUFFER_LEN 5+2+1 // 5 digits for 16 bit number + oz for ounces + null terminator

typedef struct WeightDisplay {
    ADC_HandleTypeDef* ps_hadc;
    uint16_t ps_adc_val;
    uint16_t ps_zero_offset;

    uint16_t display_weight;

    ILI9225 ILI9225;
    char weight_char_buffer[WEIGHT_CHAR_BUFFER_LEN];
} WeightDisplay;

void weight_display_ctor(WeightDisplay* wdisplay, ADC_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csx_port, uint16_t csx_pin, GPIO_TypeDef* resx_port, uint16_t resx_pin, GPIO_TypeDef* cmd_port, uint16_t cmd_pin) {
    if (!wdisplay) Error_Handler();
    wdisplay->ps_hadc = hadc;
    HAL_ADC_Start_IT(hadc);

    wdisplay->ps_zero_offset = 0;

    wdisplay->ILI9225.hspi = hspi;
    wdisplay->ILI9225.csx_port = csx_port;
    wdisplay->ILI9225.csx_pin = csx_pin;
    wdisplay->ILI9225.resx_port = resx_port;
    wdisplay->ILI9225.resx_pin = resx_pin;
    wdisplay->ILI9225.cmd_port = cmd_port;
    wdisplay->ILI9225.cmd_pin = cmd_pin;
    lcd_init(&wdisplay->ILI9225);
}

void weight_display_dtor(WeightDisplay* wdisplay) {
	HAL_ADC_Stop_DMA(wdisplay->ps_hadc);
}

void weight_display_credits(WeightDisplay* wdisplay){
	  fill_rectangle(&wdisplay->ILI9225, 0, 0, WIDTH, HEIGHT, COLOR_BLUE);
	  draw_string(&wdisplay->ILI9225, 20, 20, COLOR_GOLD, 2, "LazyCart");
	  draw_string(&wdisplay->ILI9225, 20, 55, COLOR_GOLD, 1, "Hudson Hall");
	  draw_string(&wdisplay->ILI9225, 20, 75, COLOR_GOLD, 1, "Owen Park");
	  draw_string(&wdisplay->ILI9225, 20, 95, COLOR_GOLD, 1, "Matthew Wang");
	  draw_string(&wdisplay->ILI9225, 20, 115, COLOR_GOLD, 1, "Weijia Wu");
	  draw_char(&wdisplay->ILI9225, 170, 65, 'M', COLOR_GOLD, 3);
}

void weight_display_clear(WeightDisplay* wdisplay) {
    fill_rectangle(&wdisplay->ILI9225, 0, 0, WIDTH, HEIGHT, COLOR_WHITE);
}

// Starts the ADC conversion. Conversion complete when HAL_ADC_ConvCpltCallback interrupt is triggered.
void weight_display_start_read_psensor(WeightDisplay* wdisplay) {
	HAL_ADC_Start_IT(wdisplay->ps_hadc);
}

void weight_display_psensor_adc_callback(WeightDisplay* wdisplay) {
	wdisplay->ps_adc_val = 4095 - HAL_ADC_GetValue(wdisplay->ps_hadc);
}

void weight_display_zero_weight(WeightDisplay* wdisplay) {
    wdisplay->ps_zero_offset = wdisplay->ps_adc_val;
}

void weight_display_show_raw_adc_val(WeightDisplay* wdisplay) {
	sprintf(wdisplay->weight_char_buffer, "%d", wdisplay->ps_adc_val - wdisplay->ps_zero_offset);
	size_t null_terminator_idx = strlen(wdisplay->weight_char_buffer);
	wdisplay->weight_char_buffer[null_terminator_idx] = 'o';
	wdisplay->weight_char_buffer[null_terminator_idx+1] = 'z';
	wdisplay->weight_char_buffer[null_terminator_idx+2] = '\0';
	weight_display_clear(wdisplay);
    draw_string(&wdisplay->ILI9225, 30, 70, COLOR_BLACK, 3, wdisplay->weight_char_buffer);
}

#endif
