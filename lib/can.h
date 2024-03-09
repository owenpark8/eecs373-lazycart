#ifndef can_h
#define can_h

#include "main.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct Can
{
	CAN_HandleTypeDef* handler;
    uint8_t device_id;
    CAN_RxHeaderTypeDef header;
    uint8_t buffer[8];
} Can;

Can* can_ctor(CAN_HandleTypeDef* handler, uint8_t device_id)
{
    Can* can = (Can*)malloc(sizeof(Can));
    if (can) {
    	can->handler = handler;
        can->device_id = device_id;
        if (HAL_CAN_ActivateNotification(can->handler, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        	Error_Handler();
        }
        if (HAL_CAN_Start(can->handler) != HAL_OK) {
        	Error_Handler();
        }
    }
    return can;
}

void can_filter_only_device_id(Can* can)
{
	CAN_FilterTypeDef filter;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = can->device_id;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x000F;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 0; // Meaningless since we are not using dual CAN peripherals

	if (HAL_CAN_ConfigFilter(can->handler, &filter) != HAL_OK) {
		Error_Handler();
	}
}

void can_dtor(Can* can)
{
	if (HAL_CAN_Start(can->handler) != HAL_OK) {
		Error_Handler();
	}
    free(can);
}

void can_receive(Can* can)
{
	if (HAL_CAN_GetRxMessage(can->handler, CAN_RX_FIFO0, &can->header, can->buffer) != HAL_OK)
	{
		Error_Handler();
	}
}

void can_send_data(Can* can, uint32_t id, uint8_t* data, uint32_t length)
{
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = length;

	uint32_t TxMailbox;

	if (HAL_CAN_AddTxMessage(can->handler, &TxHeader, data, &TxMailbox) != HAL_OK)
	{
	   Error_Handler ();
	}
}



#endif
