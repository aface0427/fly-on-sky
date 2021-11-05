//Ĭ�����ã�
#include "Drv_HWT101CT.h"

//�趨
#define HWT101CT_OFFLINE_TIME_MS  1000  //����

//ȫ�ֱ���
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
*�� �� ��: HWT101CT_Byte_Get
*����˵��: HWT101CT�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
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
			//�����ɹ�
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
*�� �� ��: HWT101CT_Data_Analysis
*����˵��: HWT101CT���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
static void HWT101CT_Data_Analysis(u8 *buf_data)
{
    s16 yaw_speed,yaw_angle ;
    if (*(buf_data+1)==0x52){
        yaw_speed = (s16)(((short)*(buf_data+7)<<8)|(short)*(buf_data+6));
        hwt101ct.yaw_speed = yaw_speed * 2000 / 32768; //��λΪ��/s
    }
    if(*(buf_data+1)==0x53){
        yaw_angle = (s16)((((short)*(buf_data+7))<<8)|((short)*(buf_data+6)));
        hwt101ct.yaw_angle = yaw_angle * 180 / 32768; //��λΪ��
				if(hwt101ct.first_angle<-490)hwt101ct.first_angle++;
				else if(hwt101ct.first_angle==-490)hwt101ct.first_angle=hwt101ct.yaw_angle;
				else hwt101ct.yaw_angle=fix_first(hwt101ct.yaw_angle,hwt101ct.first_angle);
    }
	HWT101CT_Check_Reset(); 
}

/**********************************************************************************************************
*�� �� ��: HWT101CT_Offline_Check
*����˵��: HWT101CT���߼�⣬�������Ӳ���Ƿ�����
*��    ��: ʱ�䣨���룩
*�� �� ֵ: ��
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
*�� �� ��: HWT101CT_Check_Reset
*����˵��: HWT101CT���߼�⸴λ��֤��û�е���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void HWT101CT_Check_Reset()
{
	hwt_offline_check_time = 0;
	hwt101ct.offline = 0;
}
