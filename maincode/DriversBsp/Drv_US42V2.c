#include "Drv_US42V2.h"

u8 DistanceX_buf[20];
u16 DistanceX;
#define TFMINI2_OFFLINE_TIME_MS  1000
u16 tf2_offline_check_time;
u8 tfmini2_buf[20];
_us42v2_data_st tfmini2;
void DistanceX_Byte_Get(u8 bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	DistanceX_buf[rec_sta] = bytedata;
	if(rec_sta==0)	{
		if(bytedata==0x5A)	{
			rec_sta++;
		}
		else{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)	{
		if(bytedata==0x5A){
			rec_sta++;
		}	
		else{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)	{
		if(bytedata==0x45){
			rec_sta++;
		}
		else{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		len = bytedata;
		if(len==2){
			rec_sta++;
		}		
		else{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+2))	{
		for(u8 i=0;i<len+4;i++){
			check_val += DistanceX_buf[i];
		}
		if(check_val == bytedata)		{
			tfmini2.Dist=(DistanceX_buf[4]<<8)|DistanceX_buf[5];
			rec_sta=0;
		}
		else{
			rec_sta=0;
		}		
	}
	else{	
		rec_sta++;
	}	
}