/*
 * NVF_CanBus_Conf.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Scott CJX
 */

#ifndef INC_NVF_CANBUS_CONF_H_
#define INC_NVF_CANBUS_CONF_H_


enum NVF2_Nodes
{
	VCU,
	MCC,
	MC,
	T_MAIN_A,
	T_MAIN_B,
	T_AUX,
	DL_VCU,
	DL_R2D,
	DR_VCU,
	AMSC
};

enum NVF2_Networks
{
	VCUN,
	PDLN,
	R2DN,
	MCN
};

union NVF2_CanId
{
	uint8_t node_name;
	uint8_t node_network;

	uint16_t ident;
};

struct canNodeRecord
{
	uint8_t errs_since_1000ms;
	uint32_t last_ts;
	bool inErrState;
};

//enum NVF2_VCUN_StdId
//{
//	VCU,
//	MCC,
//	AMSC,
//	DR_VCU,
//	DL_VCU
//};
//
//enum NVF2_R2DN_StdId
//{
//	VCU,
//	DL_R2D
//};
//
//enum NVF2_PDLN_StdId
//{
//	VCU,
//	MCC,
//	T_MAIN_A,
//	T_MAIN_B,
//	T_AUX
//};
//
//enum NVF2_MCN_StdId
//{
//	MC,
//	MCC
//};
//
#endif /* INC_NVF_CANBUS_CONF_H_ */
