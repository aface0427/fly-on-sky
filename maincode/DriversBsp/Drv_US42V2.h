#ifndef __DRV_TFMINI2_H
#define __DRV_TFMINI2_H


#include "SysConfig.h"

typedef struct
{
	u8 offline;
	s16 Dist;
	s16 Strength;
	s16 Temp;
}_us42v2_data_st;

extern _us42v2_data_st tfmini2;

static void US42V2_Data_Analysis(u8 *buf_data,u8 len);
static void US42V2_Check_Reset(void);

//public
void US42V2_Offline_Check(u8 dT_ms);
void US42V2_Byte_Get(uint8_t bytedata);

#endif