//默认引用：
#include "Drv_HWT101CT.h"

//设定
#define HWT101CT_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
u16 hwt_offline_check_time;
u8 hwt101ct_buf[20];
_hwt101ct_data_st hwt101ct;
s16 fix_first(s16 a,s16 b)
{
	a=a-b;
	int p;
	if(a>179)
	{
		p=179-a;
		a=-180-p;
	}
	else if(a<-179)
	{
		p=-179-a;
		a=180-p;
	}
	return a;
}
/**********************************************************************************************************
*函 数 名: HWT101CT_Byte_Get
*功能说明: HWT101CT字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void HWT101CT_Byte_Get(uint8_t bytedata)
{	
	static u8 rec_sta;
	u8 check_val=0;

	hwt101ct_buf[rec_sta] = bytedata;

	if(rec_sta==0)
	{
		if(bytedata==0x55)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==10)
	{
		//
		for(u8 i=0;i<10;i++)
		{
			check_val += hwt101ct_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			HWT101CT_Data_Analysis(hwt101ct_buf);
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
*函 数 名: HWT101CT_Data_Analysis
*功能说明: HWT101CT数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
static void HWT101CT_Data_Analysis(u8 *buf_data)
{
    s16 yaw_speed,yaw_angle ;
    if (*(buf_data+1)==0x52){
        yaw_speed = (s16)(((short)*(buf_data+7)<<8)|(short)*(buf_data+6));
        hwt101ct.yaw_speed = yaw_speed * 2000 / 32768; //单位为°/s
    }
    if(*(buf_data+1)==0x53){
        yaw_angle = (s16)((((short)*(buf_data+7))<<8)|((short)*(buf_data+6)));
        hwt101ct.yaw_angle = yaw_angle * 180 / 32768; //单位为°
				if(hwt101ct.first_angle<-490)hwt101ct.first_angle++;
				else if(hwt101ct.first_angle==-490)hwt101ct.first_angle=hwt101ct.yaw_angle;
				else hwt101ct.yaw_angle=fix_first(hwt101ct.yaw_angle,hwt101ct.first_angle);
    }
	HWT101CT_Check_Reset(); 
}

/**********************************************************************************************************
*函 数 名: HWT101CT_Offline_Check
*功能说明: HWT101CT掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void HWT101CT_Offline_Check(u8 dT_ms)
{
	if(hwt_offline_check_time<HWT101CT_OFFLINE_TIME_MS)
	{
		hwt_offline_check_time += dT_ms;
	}
	else
	{
		hwt101ct.offline = 1;
	}
	
}

/**********************************************************************************************************
*函 数 名: HWT101CT_Check_Reset
*功能说明: HWT101CT掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void HWT101CT_Check_Reset()
{
	hwt_offline_check_time = 0;
	hwt101ct.offline = 0;
}
