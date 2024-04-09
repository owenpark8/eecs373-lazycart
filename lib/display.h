#ifndef display_h
#define display_h

#include "stm32l4xx_hal.h"
#include "ILI9225.h"

#include <stdlib.h>

typedef struct Display {
    ILI9225 ILI9225;
} Display;

void display_ctor(Display* display, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csx_port, uint16_t csx_pin, GPIO_TypeDef* resx_port, uint16_t resx_pin, GPIO_TypeDef* cmd_port, uint16_t cmd_pin) {
    if (!display) Error_Handler();
    display->ILI9225.hspi = hspi;
    display->ILI9225.csx_port = csx_port;
    display->ILI9225.csx_pin = csx_pin;
    display->ILI9225.resx_port = resx_port;
    display->ILI9225.resx_pin = resx_pin;
    display->ILI9225.cmd_port = cmd_port;
    display->ILI9225.cmd_pin = cmd_pin;
    lcd_init(&display->ILI9225);

}

void display_credits(Display* display){
	  fill_rectangle(&display->ILI9225, 0, 0, WIDTH, HEIGHT, COLOR_BLUE);
	  draw_string(&display->ILI9225, 20, 20, COLOR_GOLD, 2, "LazyCart");
	  draw_string(&display->ILI9225, 20, 55, COLOR_GOLD, 1, "Hudson Hall");
	  draw_string(&display->ILI9225, 20, 75, COLOR_GOLD, 1, "Owen Park");
	  draw_string(&display->ILI9225, 20, 95, COLOR_GOLD, 1, "Matthew Wang");
	  draw_string(&display->ILI9225, 20, 115, COLOR_GOLD, 1, "Weijia Wu");
	  draw_char(&display->ILI9225, 170, 65, 'M', COLOR_GOLD, 3);
}

#endif
