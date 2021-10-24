//默认引用：
#include "Drv_TFMini_Plus.h"

//设定
#define TFMINI_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
u16 tf_offline_check_time;
u8 tfmini_buf[20];
_tfminiplus_data_st tfmini;
/**********************************************************************************************************
*函 数 名: OpenMV_Byte_Get
*功能说明: OpenMV字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void TFMini_Byte_Get(uint8_t bytedata)
{	
	static u8 rec_sta;
	u8 check_val=0;

	tfmini_buf[rec_sta] = bytedata;

	if(rec_sta==0)
	{
		if(bytedata==0x59)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(bytedata==0x59)
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
			check_val += tfmini_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			TFMini_Data_Analysis(tfmini_buf,9);
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
*函 数 名: TFMini_Data_Analysis
*功能说明: TFMini数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
static void TFMini_Data_Analysis(u8 *buf_data,u8 len)
{
    tfmini.Dist = (s16)((*(buf_data+3)<<8)|*(buf_data+2));
    tfmini.Strength = (s16)((*(buf_data+5)<<8)|*(buf_data+4));
    tfmini.Temp = (s16)((*(buf_data+3)<<7)|*(buf_data+6));
	TFMini_Check_Reset();
}

/**********************************************************************************************************
*函 数 名: TFmini_Offline_Check
*功能说明: TFmini掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void TFmini_Offline_Check(u8 dT_ms)
{
	if(tf_offline_check_time<TFMINI_OFFLINE_TIME_MS)
	{
		tf_offline_check_time += dT_ms;
	}
	else
	{
		tfmini.offline = 1;
	}
	
}

/**********************************************************************************************************
*函 数 名: TFMini_Check_Reset
*功能说明: TFMini掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void TFMini_Check_Reset()
{
	tf_offline_check_time = 0;
	tfmini.offline = 0;
}
