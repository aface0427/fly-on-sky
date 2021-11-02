#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==引用
#include "SysConfig.h"


//==定义
typedef struct
{
	//
	u8 is_invalid;
	s16 pos_y;
	s16 pos_z;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 deviation;
	u8 p_flag;
	s16 pos_x;
	s16 pos_y;
    s16 pos_z;
    s16 yaw;
	u8 dT_ms;

}_openmv_line_tracking_st;

typedef struct{
    u8 is_invalid;
	s16 pos_y;
    s16 pos_z;
}_openmv_apriltag_tracking_st;

typedef struct{
    u8 is_invalid;
	s16 Dist;
	s16 flag;
    s16 pos_y;   //0-300
}_openmv_pole_tracking_st;

typedef struct
{
	u8 offline;
	u8 mode_cmd[3];
	u8 mode_sta; // 3:apirltag识别 4:摩尔环识别 5:红色杆识别
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
    _openmv_apriltag_tracking_st at;
    _openmv_apriltag_tracking_st mol;
    _openmv_pole_tracking_st pole;
}_openmv_data_st;

//==数据声明
extern _openmv_data_st opmv;
extern _openmv_data_st opmv2;
//==函数声明

//static
static void OpenMV_Data_Analysis(uint8_t *buf_data,uint8_t len);
static void OpenMV_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);
void OpenMV_Byte_Get(uint8_t bytedata);

//static
static void OpenMV2_Data_Analysis(uint8_t *buf_data,uint8_t len);
static void OpenMV2_Check_Reset(void);

//public
void OpenMV2_Offline_Check(u8 dT_ms);
void OpenMV2_Byte_Get(uint8_t bytedata);
#endif

