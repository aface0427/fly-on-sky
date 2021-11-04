//Ĭ�����ã�
#include "Drv_OpenMV.h"
#include "Ano_Scheduler.h"
//�趨
#define OPMV_OFFLINE_TIME_MS  1000  //����

enum OpenMVmodeflag{
    Dot_Following = 1,
    Line_Following,
    AprilTag
}OpenMVmodeflagdata;
//ȫ�ֱ���
u16 offline_check_time;
u8 openmv_buf[20];
_openmv_data_st opmv;
/**********************************************************************************************************
*�� �� ��: OpenMV_Byte_Get
*����˵��: OpenMV�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV_Byte_Get(uint8_t bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	openmv_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
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
		if(bytedata==0x29)
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(1)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<31)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += openmv_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//�����ɹ�
			OpenMV_Data_Analysis(openmv_buf,len+6);
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
*�� �� ��: OpenMV_Data_Analysis
*����˵��: OpenMV���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
u8 a,b,c,d;
static void OpenMV_Data_Analysis(uint8_t *buf_data,uint8_t len)
{
	if(*(buf_data+3)==0x41)
	{
		opmv.cb.is_invalid = *(buf_data+5);
        if (!opmv.cb.is_invalid){
            opmv.cb.pos_y = 80 - (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv.cb.pos_z = 60 - (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
		else{
            opmv.cb.pos_y = 0;
            opmv.cb.pos_z = 0 ;
        }
        opmv.mode_sta = 1;
	}
	else if(*(buf_data+3)==0x42)    
	{
		opmv.lt.sta = *(buf_data+5);
		opmv.lt.angle = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
		opmv.lt.deviation = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
		opmv.lt.p_flag = *(buf_data+10);
		opmv.lt.pos_x = (s16)((*(buf_data+11)<<8)|*(buf_data+12));
		opmv.lt.pos_y = (s16)((*(buf_data+13)<<8)|*(buf_data+14));
		opmv.lt.dT_ms = *(buf_data+15);
        opmv.mode_sta = 2;
		//
	}
    else if(*(buf_data+3)==0x43)//AprilTagʶ����    
	{
        opmv.at.is_invalid = *(buf_data+5);
        if (!opmv.at.is_invalid){
            opmv.at.pos_y = -1 * (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv.at.pos_z = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
        else {
            opmv.at.pos_y = 0;
            opmv.at.pos_z = 0 ;
        }
        opmv.mode_sta = 3;
	}
    else if(*(buf_data+3)==0x44)//Ħ����ʶ��
	{
		opmv.mol.is_invalid = *(buf_data+5);
        if (!opmv.mol.is_invalid){
            opmv.mol.pos_y = 80 - (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv.mol.pos_z = 80 - (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
		else{
            opmv.mol.pos_y = 0;
            opmv.mol.pos_z = 0 ;
        }
        opmv.mode_sta = 4;
	}
    else if(*(buf_data+3)==0x45)//General
	{
		opmv.pole.is_invalid = *(buf_data+5);
        if (opmv.pole.is_invalid){
						opmv.pole.flag=	opmv.pole.is_invalid;
            opmv.pole.pos_x = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv.pole.pos_y = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
						opmv.pole.Dist = (s16)((*(buf_data+10)<<8)|*(buf_data+11));
        }
		else{
            opmv.pole.Dist = 0;
						opmv.pole.pos_x= 0;
            opmv.pole.pos_y = 0 ;
						opmv.pole.flag=0;
        }
        opmv.mode_sta = 5;
	}
	//
	OpenMV_Check_Reset();
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Offline_Check
*����˵��: OpenMV���߼�⣬�������Ӳ���Ƿ�����
*��    ��: ʱ�䣨���룩
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV_Offline_Check(u8 dT_ms)
{
	if(offline_check_time<OPMV_OFFLINE_TIME_MS)
	{
		offline_check_time += dT_ms;
	}
	else
	{
		opmv.offline = 1;
	}
	
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Check_Reset
*����˵��: OpenMV���߼�⸴λ��֤��û�е���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OpenMV_Check_Reset()
{
	offline_check_time = 0;
	opmv.offline = 0;
}

//ȫ�ֱ���
u16 offline_check_time2;
u8 openmv2_buf[20];
_openmv_data_st opmv2;
/**********************************************************************************************************
*�� �� ��: OpenMV_Byte_Get
*����˵��: OpenMV�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV2_Byte_Get(uint8_t bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	openmv2_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
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
		if(bytedata==0x29)
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(1)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<31)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += openmv2_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//�����ɹ�
			OpenMV2_Data_Analysis(openmv2_buf,len+6);
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
*�� �� ��: OpenMV_Data_Analysis
*����˵��: OpenMV���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
static void OpenMV2_Data_Analysis(uint8_t *buf_data,uint8_t len)
{
	if(*(buf_data+3)==0x41)
	{
		opmv2.cb.is_invalid = *(buf_data+5);
        if (!opmv2.cb.is_invalid){
            opmv2.cb.pos_y = 80 - (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv2.cb.pos_z = 60 - (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
		else{
            opmv2.cb.pos_y = 0;
            opmv2.cb.pos_z = 0 ;
        }
        opmv2.mode_sta = 1;
	}
	else if(*(buf_data+3)==0x42)    
	{
		opmv2.lt.sta = *(buf_data+5);
		opmv2.lt.angle = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
		opmv2.lt.deviation = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
		opmv2.lt.p_flag = *(buf_data+10);
		opmv2.lt.pos_x = (s16)((*(buf_data+11)<<8)|*(buf_data+12));
		opmv2.lt.pos_y = (s16)((*(buf_data+13)<<8)|*(buf_data+14));
		opmv2.lt.dT_ms = *(buf_data+15);
        opmv2.mode_sta = 2;
		//
	}
    else if(*(buf_data+3)==0x43)//AprilTagʶ����    
	{
        opmv2.at.is_invalid = *(buf_data+5);
        if (!opmv2.at.is_invalid){
            opmv2.at.pos_y = -1 * (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv2.at.pos_z = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
        else {
            opmv2.at.pos_y = 0;
            opmv2.at.pos_z = 0 ;
        }
        opmv2.mode_sta = 3;
	}
    else if(*(buf_data+3)==0x44)//Ħ����ʶ��
	{
		opmv2.mol.is_invalid = *(buf_data+5);
        if (!opmv2.mol.is_invalid){
            opmv2.mol.pos_y = 80 - (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv2.mol.pos_z = 80 - (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
		else{
            opmv2.mol.pos_y = 0;
            opmv2.mol.pos_z = 0 ;
        }
        opmv2.mode_sta = 4;
	}
    else if(*(buf_data+3)==0x45)//��ɫ��
	{
		opmv2.pole.is_invalid = *(buf_data+5);
        if (!opmv2.pole.is_invalid){
            opmv2.pole.Dist = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
            opmv2.pole.pos_y = 150 - (s16)((*(buf_data+8)<<8)|*(buf_data+9));
        }
		else{
            opmv2.pole.Dist = 0;
            opmv2.pole.pos_y = 0 ;
        }
        opmv2.mode_sta = 5;
	}
	//
	OpenMV2_Check_Reset();
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Offline_Check
*����˵��: OpenMV���߼�⣬�������Ӳ���Ƿ�����
*��    ��: ʱ�䣨���룩
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV2_Offline_Check(u8 dT_ms)
{
	if(offline_check_time2<OPMV_OFFLINE_TIME_MS)
	{
		offline_check_time2 += dT_ms;
	}
	else
	{
		opmv2.offline = 1;
	}
	
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Check_Reset
*����˵��: OpenMV���߼�⸴λ��֤��û�е���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OpenMV2_Check_Reset()
{
	offline_check_time2 = 0;
	opmv2.offline = 0;
}

