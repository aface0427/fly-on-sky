#ifndef __DRV_TFMINI_H
#define __DRV_TFMINI_H

//==引用
#include "SysConfig.h"


//==定义
typedef struct
{
	u8 offline;
	s16 Dist;
    s16 Strength;
    s16 Temp;
}_tfminiplus_data_st;
//==数据声明
extern _tfminiplus_data_st tfmini;

//==函数声明

//static
static void TFMini_Data_Analysis(u8 *buf_data,u8 len);
static void TFMini_Check_Reset(void);

//public
void TFmini_Offline_Check(u8 dT_ms);
void TFMini_Byte_Get(uint8_t bytedata);


#endif

