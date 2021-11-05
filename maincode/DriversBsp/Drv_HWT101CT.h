#ifndef __DRV_HWT101CT_H
#define __DRV_HWT101CT_H

//==����
#include "SysConfig.h"


//==����
typedef struct
{
	u8 offline;
	s16 yaw_angle;
  s16 yaw_speed;
	s16 first_angle;
}_hwt101ct_data_st;
//==��������
extern _hwt101ct_data_st hwt101ct;

//==��������

//static
static void HWT101CT_Data_Analysis(u8 *buf_data);
static void HWT101CT_Check_Reset(void);

//public
void HWT101CT_Offline_Check(u8 dT_ms);
void HWT101CT_Byte_Get(uint8_t bytedata);


#endif

