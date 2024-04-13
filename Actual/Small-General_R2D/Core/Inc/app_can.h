/*
 * app_can.h
 *
 *  Created on: Apr 5, 2024
 *      Author: s
 */

#ifndef INC_APP_CAN_H_
#define INC_APP_CAN_H_

#include <stdbool.h>
#include <string.h>

#include "stm32f0xx_hal_can.h"

#include "app_dbg.h"

void app_can_error_handler();
bool app_can_setup();
bool app_can_tx();
bool app_can_rx();

#endif /* INC_APP_CAN_H_ */
