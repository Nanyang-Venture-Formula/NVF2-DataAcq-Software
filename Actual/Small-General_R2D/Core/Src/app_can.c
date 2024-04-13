/*
 * app_can.c
 *
 *  Created on: Apr 5, 2024
 *      Author: s
 */

#include "app_can.h"

bool app_can_setup(CAN_HandleTypeDef *hcan, const CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterActivation = ENABLE;
	sFilterConfig->FilterBank = 0; 							// Filter bank 0 for standard IDs
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDLIST; 		// Set filter mode to mask
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT; 	// Use 32-bit filter for standard IDs
	sFilterConfig->FilterIdHigh = 0x000; 						// Set all bits to zero for mask
	sFilterConfig->FilterIdLow = 0x7FF; 					// Set all bits to zero for mask
	sFilterConfig->FilterMaskIdHigh = 0xFFFF; 				// Set all bits to one for mask
	sFilterConfig->FilterMaskIdLow = 0xFFFF; 				// Set all bits to one for mask
	sFilterConfig->FilterFIFOAssignment = CAN_RX_FIFO0; 	// Assign messages to FIFO0

	HAL_CAN_ConfigFilter(hcan, sFilterConfig);

	if(HAL_CAN_Start(hcan) != HAL_OK)
	{
		app_can_error_handler();
	}
}

void app_can_error_handler(CAN_HandleTypeDef *hcan)
{
	uint32_t can_error_code = HAL_CAN_GetError(hcan);
	while (1) {}
}

bool app_can_tx(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, uint8_t* data, uint8_t data_length, uint32_t* txMailbox)
{
	if (data_length > 0 || data_length < 0)
	{
//		unable to send
		return false;
	}

//	copy data to another array for sending
	uint8_t txData[data_length];
	memcpy(txData, data, data_length);

//	send the can message
	if (HAL_CAN_AddTxMessage(hcan, txHeader, txData, txMailbox) != HAL_OK)
	{
		app_can_error_handler(hcan);
	}

	return true;
}

bool app_can_rx()
{

}
