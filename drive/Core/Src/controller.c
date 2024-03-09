#include "main.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

Can* can;

void process_command()
{

}



void HAL_PostInit()
{
	can = can_ctor(&hcan1, 0x01);
	can_filter_only_device_id(can);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_receive(can);
	process_command();
}
