#ifndef pixy_h
#define pixy_h

#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

#include "avgqueue.h"

#define PIXY_WR 0xA8
#define PIXY_RD 0xA9

uint8_t transmit_buffer[6] = {0xae, 0xc1, 0x20, 0x2, 0x1, 0x1};

typedef struct Pixy {
    I2C_HandleTypeDef* hi2c;
    union {
        uint8_t raw_data[20];
        uint16_t data[10];
    } block_buffer;
    AvgQueue x_pos_average;
    AvgQueue block_width_average;
    AvgQueue block_height_average;
} Pixy;

void pixy_ctor(Pixy* pixy, I2C_HandleTypeDef *hi2c) {
    if (!pixy) Error_Handler();
    pixy->hi2c = hi2c;
    avgqueue_ctor(&pixy->x_pos_average);
    avgqueue_ctor(&pixy->block_width_average);
    avgqueue_ctor(&pixy->block_height_average);
    for (int i = 0; i < 5; ++i) {
        push(&pixy->x_pos_average, 0U);
        push(&pixy->block_width_average, 0U);
        push(&pixy->block_height_average, 0U);
    }
}

void get_blocks_i2c(Pixy* pixy) {
    if (!pixy) Error_Handler();
    HAL_I2C_Master_Transmit(pixy->hi2c, PIXY_WR, transmit_buffer, 6, 1000);
    HAL_I2C_Master_Receive(pixy->hi2c, PIXY_RD, pixy->block_buffer.raw_data, 20, 1000); // block should be in our buffer
    pop(&pixy->x_pos_average);
    push(&pixy->x_pos_average, (uint32_t)pixy->block_buffer.data[4]);
    pop(&pixy->block_width_average);
    push(&pixy->block_width_average, (uint32_t)pixy->block_buffer.data[6]);
    pop(&pixy->block_height_average);
    push(&pixy->block_height_average, (uint32_t)pixy->block_buffer.data[7]);
}

uint16_t get_average_x(Pixy* pixy) {
    return (uint16_t) get_average(&pixy->x_pos_average);
}

uint16_t get_x(Pixy* pixy) {
    if (!pixy) Error_Handler();
	return pixy->block_buffer.data[4];
}

uint16_t get_y(Pixy* pixy) {
    if (!pixy) Error_Handler();
    return pixy->block_buffer.data[5];
}

uint16_t get_average_block_width(Pixy* pixy) {
    return (uint16_t) get_average(&pixy->block_width_average);
}

uint16_t get_block_width(Pixy* pixy) {
    if (!pixy) Error_Handler();
    return pixy->block_buffer.data[6];
}

uint16_t get_average_block_height(Pixy* pixy) {
    return (uint16_t) get_average(&pixy->block_height_average);
}

uint16_t get_block_height(Pixy* pixy) {
    if (!pixy) Error_Handler();
    return pixy->block_buffer.data[7];
}

uint16_t get_block_angle(Pixy* pixy) {
    if (!pixy) Error_Handler();
    return pixy->block_buffer.data[8];
}



#endif
