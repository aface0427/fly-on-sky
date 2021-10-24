#ifndef __DRV_HWT101CT_H
#define __DRV_HWT101CT_H

//==引用
#include "SysConfig.h"


//==定义
typedef struct
{
	u8 offline;
	s16 yaw_angle;
    s16 yaw_speed;
}_hwt101ct_data_st;
//==数据声明
extern _hwt101ct_data_st hwt101ct;

//==函数声明

//static
static void HWT101CT_Data_Analysis(u8 *buf_data);
static void HWT101CT_Check_Reset(void);

//public
void HWT101CT_Offline_Check(u8 dT_ms);
void HWT101CT_Byte_Get(uint8_t bytedata);


#endif

