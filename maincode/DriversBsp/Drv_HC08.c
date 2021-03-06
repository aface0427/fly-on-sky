#include "Drv_HC08.h"

//设定
#define HC08_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
u16 HC08_offline_check_time;
u8 HC08_buf[20];
_HC08_data_st HC08;
/**********************************************************************************************************
*函 数 名: OpenMV_Byte_Get
*功能说明: OpenMV字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void HC08i_Byte_Get(uint8_t bytedata)
{	
	static u8 rec_sta;
	u8 check_val=0;

	HC08_buf[rec_sta] = bytedata;

	if(rec_sta==0)
	{
		if(bytedata==0x12)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==8)
	{
		//
		for(u8 i=0;i<8;i++)
		{
			check_val += HC08_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			HC08_Data_Analysis(HC08_buf,9);
			//
			rec_sta=0;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else
	{
		//	
		rec_sta++;
	}
	
}

/**********************************************************************************************************
*函 数 名: HC08_Data_Analysis
*功能说明: HC08数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
static void HC08_Data_Analysis(u8 *buf_data,u8 len)
{
    HC08.Dist = (s16)((*(buf_data+3)<<8)|*(buf_data+2));
    HC08.Strength = (s16)((*(buf_data+5)<<8)|*(buf_data+4));
    HC08.Temp = (s16)((*(buf_data+3)<<7)|*(buf_data+6));
	HC08_Check_Reset();
}

/**********************************************************************************************************
*函 数 名: HC08_Offline_Check
*功能说明: HC08掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void HC08_Offline_Check(u8 dT_ms)
{
	if(HC08_offline_check_time<HC08_OFFLINE_TIME_MS)
	{
		HC08_offline_check_time += dT_ms;
	}
	else
	{
		HC08.offline = 1;
	}
	
}

/**********************************************************************************************************
*函 数 名: HC08_Check_Reset
*功能说明: HC08掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void HC08_Check_Reset()
{
	HC08_offline_check_time = 0;
	HC08.offline = 0;
}








