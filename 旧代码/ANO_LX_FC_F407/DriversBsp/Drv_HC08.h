#ifndef __DRV_HC08_H
#define __DRV_HC08_H

//==引用
#include "SysConfig.h"

//==定义
typedef struct
{
	u8 offline;
	s16 Dist;
    s16 Strength;
    s16 Temp;
}_HC08_data_st;
//==数据声明
extern _HC08_data_st HC08;

//==函数声明

//static
static void HC08_Data_Analysis(u8 *buf_data,u8 len);
static void HC08_Check_Reset(void);

//public
void HC08_Offline_Check(u8 dT_ms);
void HC08_Byte_Get(uint8_t bytedata);


#endif

