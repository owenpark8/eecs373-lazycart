#ifndef can_h
#define can_h

#include "main.h"
#include "stdint.h"

typedef struct Can {
	CAN_HandleTypeDef* handler;
    uint8_t device_id;
    CAN_RxHeaderTypeDef header;
    uint8_t buffer[8];
};

Can* can_ctor(CAN_HandleTypeDef* handler, uint8_t device_id) {
    Can* can = (Can*)malloc(sizeof(Can));
    if (can) {
    	can->handler = handler;
        can->device_id = device_id;
        if (HAL_CAN_ActivateNotification(can->handler, CAN_IT_RX_FIFO0_MSG_PENDING, 0) != HAL_OK) {
        	Error_Handler();
        }
        if (HAL_CAN_Start(can->handler) != HAL_OK) {
        	Error_Handler();
        }
    }
    return can;
}

void can_filter_only_device_id(Can* can) {
	CAN_FilterTypeDef filter;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0; // which filter bank to use from the assigned ones
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = can->device_id;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x000F;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 0; // Meaningless since we are not using dual CAN peripherals

	if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
		Error_Handler();
	}
}

void can_dtor(Can* can) {
	if (HAL_CAN_Start(can->handler) != HAL_OK) {
		Error_Handler();
	}
    free(can);
}

void can_receive(Can* can) {
	if (HAL_CAN_GetRxMessage(can->handler, CAN_RX_FIFO0, &can->header, can->buffer) != HAL_OK)
	{
		Error_Handler();
	}
}

void can_send_data(Can* can, ) {
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x446;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;


	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	   Error_Handler ();
	}
}


#endif
