/*
 * app_can_impl.h
 *
 *  Created on: Apr 5, 2024
 *      Author: s
 */

#ifndef INC_APP_CAN_IMPL_H_
#define INC_APP_CAN_IMPL_H_


//for R2DN
#define R2DN_VCU_CANID			(uint32_t)(0x0010)
#define R2DN_DASH_CANID			(uint32_t)(0x0011)


typedef struct _DASH_R2D_CANF
{
	bool hasError;
	bool isDeactivated;
	bool isTriggered;

} DASH_R2D_CANF;

bool get_DASH_R2D_CANF(uint8_t *data, DASH_R2D_CANF* ret)
{
	ret.hasError 			= (data & 0b1000);
	ret.isDeactivated 		= (data & 0b0100);
	ret.isTriggered 		= (data & 0b0010);
	return 1;
}

bool get_DASH_R2D_CANF(DASH_R2D_CANF *data, uint8_t *ret)
{
	&ret = 0x00;
	&ret  |= (data.hasError << 3);
	&ret  |= (data.isDeactivated << 2);
	&ret  |= (data.isTriggered << 1);
	return 1;
}



#endif /* INC_APP_CAN_IMPL_H_ */
