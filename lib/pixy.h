#ifndef pixy_h
#define pixy_h

#include "main.h"
#include <stdlib.h>
#include <stdint.h>

#define PIXY_WR 0xA8
#define PIXY_RD 0xA9

uint8_t transmit_buffer[6] = {0xae, 0xc1, 0x20, 0x2, 0x1, 0x1};

typedef struct Pixy {
    I2C_HandleTypeDef* hi2c;
    union {
        uint8_t raw_data[20];
        uint16_t data[10];
    } block_buffer;
} Pixy;

Pixy* pixy_ctor(I2C_HandleTypeDef *hi2c) {
    Pixy* pixy = (Pixy*)malloc(sizeof(Pixy));
    if (!pixy) Error_Handler();
    pixy->hi2c = hi2c;
    return pixy;
}

void pixy_dtor(Pixy* pixy) {
    free(pixy);
}

void get_blocks_i2c(Pixy* pixy) {
	  HAL_I2C_Master_Transmit(pixy->hi2c, PIXY_WR, transmit_buffer, 6, 1000);
	  HAL_I2C_Master_Receive(pixy->hi2c, PIXY_RD, pixy->block_buffer.raw_data, 20, 1000); // block should be in our buffer
}

uint16_t get_x(Pixy* pixy) {
	return pixy->block_buffer.data[4];
}

uint16_t get_y(Pixy* pixy) {
    return pixy->block_buffer.data[5];
}

uint16_t get_block_width(Pixy* pixy) {
    return pixy->block_buffer.data[6];
}

uint16_t get_block_height(Pixy* pixy) {
    return pixy->block_buffer.data[7];
}

uint16_t get_block_angle(Pixy* pixy) {
    return pixy->block_buffer.data[8];
}



#endif
