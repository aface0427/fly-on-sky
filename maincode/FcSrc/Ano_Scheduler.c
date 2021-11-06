/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ���������
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "Drv_OpenMV.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_US42V2.h"
#include "Drv_TFMini_Plus.h"
#include "Drv_HWT101CT.h"
#include "User_Task.h"
#include "LX_FC_Fun.h"
#include "Drv_OpenMV.h"
#include "Drv_Uart.h"
extern _ano_of_st ano_of;
const int common_time=2500;
const int common_speed=20;
_user_flag_set user_flag = {0};
s16 dx, dy;
u16 cnt = 0;
u16 flag=0;
u8 user_send_buffer[50];
s16 pp;
u8 num=0;
u8 OpenmvSend[5]={0};
static void ANO_DT_LX_Send_Data(u8 *dataToSend, u8 length)
{
	//
	UartSendLXIMU(dataToSend, length);
}
static u8 change1(s16 a)
{
	return a&((1<<8)-1);
}
static u8 change2(s16 a)
{
	return (a^change1(a))>>8;
}
int myabs(int x)
{
	if(x<0)return -x;
	return x;
}
static void user_send(u8 frame_num,s16 data1,s16 data2)
{
	u8 cnt=0;
	user_send_buffer[cnt++]=0xAA;
	user_send_buffer[cnt++]=0xff;
	user_send_buffer[cnt++]=frame_num;
	user_send_buffer[cnt++]=4;
	user_send_buffer[cnt++]=change1(data1);
	user_send_buffer[cnt++]=change2(data1);
	user_send_buffer[cnt++]=change1(data2);
	user_send_buffer[cnt++]=change2(data2);
	u8 check_sum1 = 0, check_sum2 = 0;
	for (u8 i = 0; i < cnt; i++)
	{
		check_sum1 += user_send_buffer[i];
		check_sum2 += check_sum1;
	}
	user_send_buffer[cnt++] = check_sum1;
	user_send_buffer[cnt++] = check_sum2;
	ANO_DT_LX_Send_Data(user_send_buffer, cnt);
}
//��ͨ�˲����Բ���
s16 dis_fix_x, dis_fix_y;
s16 dis_x, dis_y;

/*xy�ٶȻ�����*/
_PID_arg_st PID_Speed_arg_x; 
_PID_arg_st PID_Speed_arg_y; 
/*xy�ٶȻ�λ��*/
_PID_val_st PID_Speed_val_x;
_PID_val_st	PID_Speed_val_y;

/*xyλ�û�����*/
_PID_arg_st PID_Distance_arg_x;
_PID_arg_st PID_Distance_arg_y; 
/*xyλ�û�����*/
_PID_val_st PID_Distance_val_x;
_PID_val_st	PID_Distance_val_y;

/*yaw�ٶȻ�����*/
_PID_arg_st PID_Speed_arg_yaw;
/*yaw�ٶȻ�����*/
_PID_val_st PID_Speed_val_yaw;

/*yawλ�û�����*/
_PID_arg_st PID_Distance_arg_yaw;
/*yawλ�û�����*/
_PID_val_st PID_Distance_val_yaw;

/*z�ٶȻ�����*/
_PID_arg_st PID_Speed_arg_z;
/*z�ٶȻ�����*/
_PID_val_st PID_Speed_val_z;

/*zλ�û�����*/
_PID_arg_st PID_Distance_arg_z;
/*zλ�û�����*/
_PID_val_st PID_Distance_val_z;

/*x����*/
_user_exp_fdb_set user_exp_fdb_x;
/*x��ֵ*/
_user_threshold_set user_threshold_x;
/*x�������*/
s16 test_output_x;

/*�Ƹ�x��ֵ*/
_user_threshold_set user_threshold_pole_x;
/*�Ƹ�x�������*/
s16 test_output_pole_x;


/*y����*/
_user_exp_fdb_set user_exp_fdb_y;
/*y��ֵ*/
_user_threshold_set user_threshold_y;
/*y�������*/
s16 test_output_y;

/*z����*/
_user_exp_fdb_set user_exp_fdb_z;
/*z��ֵ*/
_user_threshold_set user_threshold_z;
/*z�������*/
s16 test_output_z;

/*��������*/
_user_exp_fdb_set user_exp_fdb_alt_z;
/*������ֵ*/
_user_threshold_set user_threshold_alt_z;
/*���߲������*/
s16 test_output_alt_z;

/*yaw����*/
_user_exp_fdb_set user_exp_fdb_yaw;
/*yaw��ֵ*/
_user_threshold_set user_threshold_yaw;
/*yaw�������*/
s16 test_output_yaw;

/*imuŷ��������*/
_user_eula_set user_eula;

s16 Position_now;
s16 Position_pre;
s16 Position_incre;
int test11,test22;
static s16 pos_now;
static s16 pos_pre;
static s16 pos_incre;
static s16 pos_start;
int p=1000;
int x=0,y=0;
const int xx=90,yy=90;
void change_back()
{
	x=xx;
	y=yy;
}
bool timejudge(int s)
{
	if(cnt/p>=s)return 1;
	return 0;
}
int change_posx(int x)
{
	if(opmv.pole.pos_x<0)return -x;
	return x;
}
int change_posy(int y)
{
	if(opmv.pole.pos_y<0)return -y;
	return y;
	
}
//////////////////////////////////////////////////////////////////////
//�û����������
//////////////////////////////////////////////////////////////////////

static void Loop_1000Hz(void) //1msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_100Hz(void) //10msִ��һ��
{
	
	
}

static void Loop_50Hz(void) //20msִ��һ��
{
	
	//////////////////////////////////////////////////////////////////////
  OpenMV_Offline_Check(20);
  TFmini_Offline_Check(20);
	UserTask_OneKeyCmd();
	//////////////////////////////////////////////////////////////////////
	u8 _dt = 20;
	static s32 dis_dx, dis_dy;
	static s32 _dis_x, _dis_y;         
	
	/*�ٶȻ���*/
	if(user_flag.of_dis_clear_cmd){
		dis_dx = 0;
		dis_dy = 0;
	}
	else{
		dis_dx += _dt * ano_of.of2_dx_fix;
		dis_dy += _dt * ano_of.of2_dy_fix;
	}
	
	/*��ͨ�˲�*/
	_dis_x += _dt * ano_of.of1_dx;
	_dis_y += _dt * ano_of.of1_dy;
	dis_x = _dis_x / 1000;
	dis_y = _dis_y / 1000;
	dis_fix_x += (dis_x - dis_fix_x) * 0.2;
	dis_fix_y += (dis_y - dis_fix_y) * 0.2;
	
	/*ʵ�ʾ�������*/
	dx = dis_dx / 1000;
	dy = dis_dy / 1000;
	
	dt.fun[0xf1].WTS = 1;
	dt.fun[0xf2].WTS = 1;
	dt.fun[0xf3].WTS = 1;
//		/*���ݷ���*/
	//user_send(0xf1,mission_step,0);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50msִ��һ��
{
	user_send(0xf1,test_output_x,test_output_y);
	user_send(0xf2,opmv.pole.pos_x,opmv.pole.pos_y);
	user_send(0xf4,cnt,mission_step);
	user_send(0xf3,opmv.pole.flag,hwt101ct.yaw_angle);
	/*********************************�Ƹ�*******************************************/
	if(user_flag.pole_ctl_flag&&opmv.pole.is_invalid&&opmv.pole.pos_y>30&&opmv.pole.pos_y<70){
		PolePosCtl(50, -10, 0); //x���򱣳־��롢y�����ƶ��ٶȡ�yaw����λ��
	}
	else if(user_flag.pole_ctl_flag)PolePosCtl(50, 0, 0);
	
	/*********************************OpenMV yz�ᶨλ*******************************************/
	//user_flag.opmv_ctl_flag=1;
	if(user_flag.opmv_ctl_flag){
		OpMVPosCtl_pole(0, 0); //y����������z��������
	}
	test11=rt_tar.st_data.vel_y;
	/*********************************HWT101 yaw�ᶨλ*******************************************/
	if(user_flag.hwt101_ctl_flag){
		//HWT101PosCtl(0); //���ò���
		//RealTimeSpeedControl(-15,Direction_yaw);
		angle_fix();
	}
	/*********************************TFmini x�ᶨλ*******************************************/
  if(user_flag.tfmini_ctl_flag){
		TFMiniPosCtl(50); //x���򶨾���
	}	
	
	
	/*********************************�������ⶨ��*******************************************/
	if(user_flag.of_alt_ctl_flag==1){
		OFAltCtl(150); //�����߶�
	}
	else if(user_flag.of_alt_ctl_flag==2){
		OFAltCtl(10); //�����߶�
	}
	/*********************************����*******************************************/
	if(mission_task){
		//TaskSet(50);
		//taskset2(50);
		//taskset3(50);
		//realtask(50);
	}
	/*********************************openmv xy�ᶨλ*******************************************/
	//user_flag.openmv_down_flag=1;
  if(user_flag.openmv_down_flag==1){
		OpMVPosCtl_Down(x,y); //x,y���򶨾���
	}	
	else if(user_flag.openmv_down_flag==2)//y���򶨾���
	{
		OpMVPosCtl_Down(-500,y);
	}
	test22=rt_tar.st_data.vel_y;
	/*********************************����λ����*******************************************/
	if(user_flag.openmv_clr_flag){
		DataClr();
		user_flag.openmv_clr_flag = 0;
	}
	
	/*����ʵʱ����֡*/
		
		dt.fun[0x41].WTS = 1;
	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500msִ��һ��
{
//    opmv.mode_cmd[0] = 0x0A;
//    opmv.mode_cmd[1] = 2;
//    opmv.mode_cmd[2] = opmv.mode_cmd[1] + opmv.mode_cmd[0];
//    DrvUart1SendBuf(&opmv.mode_cmd[0],3);
			
}

/*
*@fn:			void Init_PID(void)
*@brief:	PID������ʼ��
*@para:		none
*@return:	none
*@comment:
*/
void Init_PID(void){
//x�ٶȻ�
	PID_Speed_arg_x.kp = 0.5f;
	PID_Speed_arg_x.ki = 0;
	PID_Speed_arg_x.kd_ex = 0;
	PID_Speed_arg_x.fb_d_mode = 0;
	PID_Speed_arg_x.kd_fb = 0;
	PID_Speed_arg_x.k_ff = 0;
	
//xλ�û�
	PID_Distance_arg_x.kp = 2.0f;
	PID_Distance_arg_x.ki = 0;
	PID_Distance_arg_x.kd_ex = 0;
	PID_Distance_arg_x.fb_d_mode = 0;
	PID_Distance_arg_x.kd_fb = 0;
	PID_Distance_arg_x.k_ff = 0;
	
//y�ٶȻ�
	PID_Speed_arg_y.kp = 0.5f;
	PID_Speed_arg_y.ki = 0;
	PID_Speed_arg_y.kd_ex = 0;
	PID_Speed_arg_y.fb_d_mode = 0;
	PID_Speed_arg_y.kd_fb = 0;
	PID_Speed_arg_y.k_ff = 0;
	
//yλ�û�
	PID_Distance_arg_y.kp = 2.0f;
	PID_Distance_arg_y.ki = 0;
	PID_Distance_arg_y.kd_ex = 0;
	PID_Distance_arg_y.fb_d_mode = 0;
	PID_Distance_arg_y.kd_fb = 0;
	PID_Distance_arg_y.k_ff = 0;	
	
//yaw�ٶȻ�
	PID_Speed_arg_yaw.kp = 0.5f;
	PID_Speed_arg_yaw.ki = 0;
	PID_Speed_arg_yaw.kd_ex = 0;
	PID_Speed_arg_yaw.fb_d_mode = 0;
	PID_Speed_arg_yaw.kd_fb = 0;
	PID_Speed_arg_yaw.k_ff = 0;
	
//yawλ�û�
	PID_Distance_arg_yaw.kp = 6.0f;
	PID_Distance_arg_yaw.ki = 0;
	PID_Distance_arg_yaw.kd_ex = 0;
	PID_Distance_arg_yaw.fb_d_mode = 0;
	PID_Distance_arg_yaw.kd_fb = 0;
	PID_Distance_arg_yaw.k_ff = 0;	
	
//z�ٶȻ�
	PID_Speed_arg_z.kp = 0.5f;
	PID_Speed_arg_z.ki = 0;
	PID_Speed_arg_z.kd_ex = 0;
	PID_Speed_arg_z.fb_d_mode = 0;
	PID_Speed_arg_z.kd_fb = 0;
	PID_Speed_arg_z.k_ff = 0;
	
//zλ�û�
	PID_Distance_arg_z.kp = 3.0f;
	PID_Distance_arg_z.ki = 0;
	PID_Distance_arg_z.kd_ex = 0;
	PID_Distance_arg_z.fb_d_mode = 0;
	PID_Distance_arg_z.kd_fb = 0;
	PID_Distance_arg_z.k_ff = 0;	
}

/*
*@fn:			void Init_GeneralCtlArg(void)
*@brief:	ͨ�ÿ��Ʋ�����ʼ��
*@para:		none
*@return:	none
*@comment:
*/
void Init_GeneralCtlArg(void){
	/*x*/
	user_threshold_x.max_speed = 15;
	user_threshold_x.normalize_distance = 500.0f;
	user_threshold_x.normalize_speed = 20.0f;
	
	/*x�Ƹ�*/
	user_threshold_pole_x.max_speed = 3;
	user_threshold_pole_x.normalize_distance = 60.0f;
	user_threshold_pole_x.normalize_speed = 3.0f;
	
	/*y*/
	user_threshold_y.max_speed = 15;
	user_threshold_y.normalize_distance = 80.0f;
	user_threshold_y.normalize_speed = 20.0f;
	
	/*z*/
	user_threshold_z.max_speed = 15;
	user_threshold_z.normalize_distance = 40.0f;
	user_threshold_z.normalize_speed = 15.0f;
	
	/*����*/
	user_threshold_alt_z.max_speed = 15;
	user_threshold_alt_z.normalize_distance = 50.0f;
	user_threshold_alt_z.normalize_speed = 15.0f;
	
	/*yaw*/
	user_threshold_yaw.max_speed = 30;
	user_threshold_yaw.normalize_distance = 200.0f;
	user_threshold_yaw.normalize_speed = 15.0f;
}

//////////////////////////////////////////////////////////////////////
//��������ʼ��
//////////////////////////////////////////////////////////////////////
//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},
		{Loop_500Hz, 500, 0, 0},
		{Loop_200Hz, 200, 0, 0},
		{Loop_100Hz, 100, 0, 0},
		{Loop_50Hz, 50, 0, 0},
		{Loop_20Hz, 20, 0, 0},
		{Loop_2Hz, 2, 0, 0},
};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for (index = 0; index < TASK_NUM; index++)
	{
		//����ÿ���������ʱ������
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//�������Ϊ1��Ҳ����1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��

	for (index = 0; index < TASK_NUM; index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = GetSysRunTimeMs();
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
		}
	}
}

void angle_fix(void)
{
	user_exp_fdb_yaw.exp_distance=user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle;
	if(hwt101ct.yaw_angle>-40&&hwt101ct.yaw_angle<40)
	{
		user_exp_fdb_yaw.exp_distance=359;
		user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle+359;
		test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
	}
	else if(hwt101ct.yaw_angle>50&&hwt101ct.yaw_angle<130)
	{
		user_exp_fdb_yaw.exp_distance=359+90;
		user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle+359;
		test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
	}
	else if(hwt101ct.yaw_angle>-130&&hwt101ct.yaw_angle<-50)
	{
		user_exp_fdb_yaw.exp_distance=359-90;
		user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle+359;
		test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
	}
	else if(hwt101ct.yaw_angle>140||hwt101ct.yaw_angle<-140)
	{
		if(hwt101ct.yaw_angle>140)
		{
			user_exp_fdb_yaw.exp_distance=180;
			user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle;
			test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 0);
		}
		else if(hwt101ct.yaw_angle<-140)
		{
			user_exp_fdb_yaw.exp_distance=-180;
			user_exp_fdb_yaw.fdb_distance=hwt101ct.yaw_angle;
			test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
		}
	}
}
u8 TFMiniPosCtl(s16 expect){
	user_exp_fdb_x.exp_distance = expect;
	if(mission_step==8&&tfmini.Dist>100)user_exp_fdb_x.fdb_distance = expect;
	else user_exp_fdb_x.fdb_distance = tfmini.Dist;
	if(opmv.pole.is_invalid)user_exp_fdb_x.fdb_distance=opmv.pole.pos_y;
	test_output_x = GeneralPosCtl(user_exp_fdb_x, Direction_x, PID_Distance_arg_x, PID_Distance_val_x, user_threshold_x, 1);
	return 1;
}

u8 OpMVPosCtl(s16 expect1, s16 expect2){
	user_exp_fdb_z.exp_distance = expect2;
	user_exp_fdb_y.exp_distance = expect1;
		
	if(opmv.at.is_invalid){
		user_exp_fdb_y.fdb_distance = user_exp_fdb_y.exp_distance;
		user_exp_fdb_z.fdb_distance = user_exp_fdb_z.exp_distance;
	}
	else{
		user_exp_fdb_y.fdb_distance = opmv.at.pos_y;
		user_exp_fdb_z.fdb_distance = opmv.at.pos_z;
	}
		
	test_output_y = GeneralPosCtl(user_exp_fdb_y, Direction_y, PID_Distance_arg_y, PID_Distance_val_y, user_threshold_y, 1);
	test_output_z = GeneralPosCtl(user_exp_fdb_z, Direction_z, PID_Distance_arg_z, PID_Distance_val_z, user_threshold_z, 1);
	
	return 1;
}
u8 OpMVPosCtl_Down(s16 expect1, s16 expect2){
	user_exp_fdb_x.exp_distance = expect1;
	user_exp_fdb_y.exp_distance = expect2;
		
	if(opmv.pole.flag==1){
		
		user_exp_fdb_x.fdb_distance = opmv.pole.pos_x;
		user_exp_fdb_y.fdb_distance = opmv.pole.pos_y;
	}
	else{
		user_exp_fdb_x.fdb_distance = user_exp_fdb_x.exp_distance;
		user_exp_fdb_y.fdb_distance = user_exp_fdb_y.exp_distance;
	}
	if(expect1!=-500)
	test_output_x = GeneralPosCtl(user_exp_fdb_x, Direction_x, PID_Distance_arg_x, PID_Distance_val_x, user_threshold_y, 1);
	test_output_y = GeneralPosCtl(user_exp_fdb_y, Direction_y, PID_Distance_arg_x, PID_Distance_val_y, user_threshold_y, 0);
	
	return 1;
}
u8 OpMVPosCtl_pole(s16 expect1, s16 expect2){
	//user_exp_fdb_z.exp_distance = expect2;
	user_exp_fdb_y.exp_distance = expect1;
		
	if(!opmv.pole.flag){
		user_exp_fdb_y.fdb_distance = user_exp_fdb_y.exp_distance;
		//user_exp_fdb_z.fdb_distance = user_exp_fdb_z.exp_distance;
	}
	else{
		user_exp_fdb_y.fdb_distance = opmv.pole.Dist;
		//user_exp_fdb_z.fdb_distance = opmv.pole.pos_y;
	}

	test_output_y = GeneralPosCtl(user_exp_fdb_y, Direction_y, PID_Distance_arg_y, PID_Distance_val_y, user_threshold_y, 1);
	//test_output_z = GeneralPosCtl(user_exp_fdb_z, Direction_z, PID_Distance_arg_z, PID_Distance_val_z, user_threshold_z, 1);
	if(myabs(test_output_y)<=5)user_flag.tfmini_ctl_flag=1;
	else user_flag.tfmini_ctl_flag=0,rt_tar.st_data.vel_x=0;
	return 1;
}
u8 HWT101PosCtl(s16 expect){
	/*hwt101��֤yaw��ƽ��*/
	if(user_flag.yaw_set_flag){
		user_exp_fdb_yaw.exp_distance = hwt101ct.yaw_angle;
		user_flag.yaw_set_flag = 0;
	}
	
	if(hwt101ct.offline){
		user_exp_fdb_yaw.fdb_distance = user_exp_fdb_yaw.exp_distance;
	}
	else{
	/*������ж�*/
		Position_now = hwt101ct.yaw_angle;
		if((Position_now - Position_pre) > 180)
		{
			Position_incre  += (Position_now - Position_pre) - 359;
		}
		else if((Position_now - Position_pre) < -180)
		{
			Position_incre  += 359 + (Position_now - Position_pre);
		}
		else
		{
			Position_incre  += (Position_now - Position_pre);
		}
		Position_pre = Position_now;
			
		user_exp_fdb_yaw.fdb_distance = Position_incre;
	}
	test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
	
	return 1;
}	

u8 PolePosCtl(s16 exp_x, s16 exp_y, s16 exp_yaw){	
	user_exp_fdb_x.exp_distance = exp_x;
	user_exp_fdb_yaw.exp_distance = exp_yaw;
		
	if(!opmv.pole.is_invalid){
		//user_exp_fdb_x.fdb_distance = user_exp_fdb_x.exp_distance;
		user_exp_fdb_yaw.fdb_distance = user_exp_fdb_yaw.exp_distance;
	}
	else{
		//user_exp_fdb_x.fdb_distance = opmv.pole.Dist;
		user_exp_fdb_yaw.fdb_distance = opmv.pole.Dist;
	}
			
	//test_output_x = GeneralPosCtl(user_exp_fdb_x, Direction_x, PID_Distance_arg_x, PID_Distance_val_x, user_threshold_pole_x, 1);
	test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
	rt_tar.st_data.vel_y = exp_y;	
	if(opmv.pole.Dist>-30||opmv.pole.Dist<30)user_flag.tfmini_ctl_flag=1;
	else user_flag.tfmini_ctl_flag=0,rt_tar.st_data.vel_x=0;
	return 1;
}
u8 OFAltCtl(u16 expect){
	user_exp_fdb_alt_z.exp_distance = expect;
	user_exp_fdb_alt_z.fdb_distance = ano_of.of_alt_cm;	
	//user_send(0xf2,test1,test2);
	if(user_exp_fdb_alt_z.fdb_distance < 30)
		rt_tar.st_data.vel_z = 0;
	else
		test_output_alt_z = GeneralPosCtl(user_exp_fdb_alt_z, Direction_z, PID_Distance_arg_z, PID_Distance_val_z, user_threshold_alt_z, 0);
	//user_send(0xf3,Direction_z,ano_of.of_alt_cm);
	//user_send(0xf3,test_output_alt_z,ano_of.of_alt_cm);
	return 1;
}
u8 DataClr(void){
	
	RealTimeSpeedControl(0, Direction_x);
	RealTimeSpeedControl(0, Direction_y);
	RealTimeSpeedControl(0, Direction_z);
	RealTimeSpeedControl(0, Direction_yaw);
	Position_incre = 0;
	Position_pre = 0;
	pos_incre = 0;
	pos_pre = 0;
	cnt = 0;
	//RealTimeSpeedControlSend(0, Direction_yaw);
	
	return 1;
}	

u8 realtask(s16 dT)
{
	switch(mission_step)
	{
		case 0:
			cnt=0;
			break;
		case 1:
			FC_Unlock();
			cnt+=dT;
			if(timejudge(3))
			{
				mission_step++;
				cnt=0;
			}
			break;
		case 2:
			mission_step+=OneKey_Takeoff(150);
			angle_fix();
			
			OpenmvSend[0]=0x08;
			OpenmvSend[1]=(u8)30;
			OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];			
			DrvUart1SendBuf(&OpenmvSend[0], 3);
			if(opmv.pole.flag==4)
			{
				x=0,y=0;
				/*��ɵ㶨λ*/
				user_flag.openmv_down_flag=1;
			}
			else
			{
				x=0,y=0;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 3:
			user_flag.of_alt_ctl_flag=1;
			angle_fix();
			if(opmv.pole.flag==4)
			{
				x=0,y=0;
				/*��ɵ㶨λ*/
				user_flag.openmv_down_flag=1;
			}
			else
			{
				x=0,y=0;
				user_flag.openmv_down_flag=0;
			}
			if(ano_of.of_alt_cm>145&&ano_of.of_alt_cm<155)
			{
				cnt+=dT;
			}
			else cnt=0;
			OpenmvSend[0]=0x08;
			OpenmvSend[1]=(u8)30;
			OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];			
			DrvUart1SendBuf(&OpenmvSend[0], 3);
			if(timejudge(3))
			{
				mission_step=100;
				DataClr();
				cnt=0;
				Position_incre=0;
				Position_pre=0;
			}
			break;
		case 100:
			angle_fix();
			if(opmv.pole.flag==4)
			{
				x=0;y=0;
				/*��ɵ㶨λ,���+-10���ص�*/
				user_flag.openmv_down_flag=1;
				if(UserAbs(opmv.pole.pos_x-x)<=10&&UserAbs(opmv.pole.pos_y-y)<=10)
				cnt+=dT;
			}
			else
			{
				x=0,y=0;
				user_flag.openmv_down_flag=0;
			}
			if(timejudge(5))
			{
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)29;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];			
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				DataClr();
				mission_step++;
				cnt=0;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 101://��A x���
			angle_fix();
			RealTimeSpeedControl(20,Direction_x);
			cnt+=dT;
		  if(cnt>=9500)
			{
				mission_step++;
				DataClr();
				cnt=0;
				Position_incre=0;
				Position_pre=0;
			}
		case 102://��A y���
			angle_fix();
			RealTimeSpeedControl(-10,Direction_y);
			if(opmv.pole.flag==3)cnt+=dT;
			OpenmvSend[0]=0x08;
			OpenmvSend[1]=(u8)31;
			OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];			
			DrvUart1SendBuf(&OpenmvSend[0], 3);
			if(cnt>=1000)
			{
				mission_step=4;
				DataClr();
				cnt=0;
				Position_incre=0;
				Position_pre=0;
			}
		case 4:/*21��A��λ x��*/
			angle_fix();
			if(opmv.pole.flag==3)
			{
				x=-33;y=-80;
				/*δת��A��λ��δ����,���+-10���ص�*/
				user_flag.openmv_down_flag=1;
				if(UserAbs(opmv.pole.pos_x-x)<=10&&UserAbs(opmv.pole.pos_y-y)<=10)
				cnt+=dT;
			}
			else
			{
				x=0,y=0;
				user_flag.openmv_down_flag=0;
			}
			if(timejudge(5))
			{
				DataClr();
				mission_step++;
				cnt=0;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 5:/*21����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 6:/*21����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(-20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		case 7:/*21�ų���y����������*/
			angle_fix();
			if(opmv.pole.flag==1)
			{
				x=-40;y=20;
				/*ת���A��λ������y����δ����*/
				user_flag.openmv_down_flag=1;
				if(UserAbs(opmv.pole.pos_x-x)<=10&&UserAbs(opmv.pole.pos_y-y)<=10)
				cnt+=dT;
			}
			else
			{
				x=0,y=0;
				user_flag.openmv_down_flag=0;
			}
			if(cnt>3000)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)21;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];			
				DrvUart1SendBuf(&OpenmvSend[0], 3);
			}
			if(cnt>4000)
			{
				DataClr();
				mission_step++;
				angle_fix();
				cnt=0;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 8:/*ǰ����20*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)20;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 9:/*ǰ����19*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)19;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 10:/*ǰ����18*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)18;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 11:/*18����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 12:/*18����ת ����x��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(-20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		
		case 13:/*ǰ����14*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)14;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 14:/*ǰ����10*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)10;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 15:/*ǰ����8*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)8;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 16:/*ǰ����4*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)4;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 17:/*4����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 18:/*4����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
			
		case 19:/*ǰ����3*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)3;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 20:/*ǰ����2*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)2;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 21:/*ǰ����1*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)1;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 22:/*1����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 23:/*1����ת ����x��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		
		case 24:/*ǰ����5*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)5;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 25:/*5����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 26:/*5����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
			
		case 27:/*ǰ����6*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)6;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 28:/*ǰ����7*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)7;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 29:/*7����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 30:/*7����ת ����x��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(-20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
			
		case 31:/*ǰ����9*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)9;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 32:/*ǰ����13*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)13;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 33:/*ǰ����17*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)17;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 34:/*ǰ����24*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)24;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 35:/*24����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 36:/*24����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		case 37:/*������23*/
			change_back();
			RealTimeSpeedControl(-20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)23;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 38:/*������22*/
			change_back();
			RealTimeSpeedControl(-20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)22;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 39:/*22����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 40:/*22����ת ����x��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(-20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		
		case 41:/*������15*/
			change_back();
			RealTimeSpeedControl(-20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)15;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 42:/*������11*/
			change_back();
			RealTimeSpeedControl(-20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)11;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 43:/*11����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 44:/*11����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(-20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		case 45:/*������12*/
			change_back();
			RealTimeSpeedControl(-20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)12;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 46:/*12����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 47:/*12����ת ����x��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
		
		case 48:/*ǰ����16*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)16;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 49:/*ǰ����23*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)23;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		case 50:/*23����ת׼��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;
			break;
		case 51:/*23����ת ����y��*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			RealTimeSpeedControl(20, Direction_yaw);
			pos_pre = pos_now;
			if(UserAbs(pos_incre-pos_start)>80){
				flag=1;
				angle_fix();
			}
			if(flag==1)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
				cnt+=dT;
			}
			if(timejudge(2))
			{
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				flag=0;
				DataClr();
				cnt=0;
			}
			break;
			
		case 52:/*ǰ����24*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)24;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 53:/*ǰ����25*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)25;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 54:/*ǰ����26*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)26;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
		
		case 55:/*ǰ����27*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)27;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
			
		case 56:/*ǰ����28*/
			change_back();
			RealTimeSpeedControl(20,Direction_x);
			angle_fix();
			cnt+=dT;
			if(cnt>=2500)
			{
				DataClr();
				flag=1;
			}
			if(flag==1)
			{
				angle_fix();
				OpenmvSend[0]=0x08;
				OpenmvSend[1]=(u8)28;
				OpenmvSend[2]=OpenmvSend[0]+OpenmvSend[1];
				DrvUart1SendBuf(&OpenmvSend[0], 3);
				if(cnt>=3500)flag=2;
			}
			if(flag==2)
			{
				angle_fix();
				if(opmv.pole.flag==1)
				{
					x=change_posx(x);
					y=change_posy(y);
					user_flag.openmv_down_flag=1;
				}
				else if(opmv.pole.flag==2)
				{
					y=change_posy(y);
					user_flag.openmv_down_flag=2;
				}
				else if(opmv.pole.flag==0)
				{
					user_flag.openmv_down_flag=0;
				}
			}
			if(cnt>=5000)
			{
				DataClr();
				angle_fix();
				flag=0;
				cnt=0;
				mission_step++;
				user_flag.openmv_down_flag=0;
			}
			break;
	}
}
/*ending part
case 8:
			user_flag.of_alt_ctl_flag = 0;
			mission_step+=OneKey_Land();
			break;
		case 9:
			cnt+=dT;
			if(timejudge(3))
			{
				cnt=0;
				mission_step++;
			}
			break;
		case 10:
			FC_Lock();
			mission_step =0;
			break;
		default:
			break;*/

s16 UserAbs(s16 num){
	if(num < 0)
		return -1 * num;
	else	
		return num;
}
s16 ZeroPointCross(s16 pos_now, s16 pos_pre, s16 pos_incre){
	if((pos_now - pos_pre) > 180)
	{
		pos_incre  += -179-pos_pre+pos_now-180;
	}
	else if((pos_now - pos_pre) < -180)
	{
		pos_incre  += 359 + (pos_now - pos_pre);
	}
	else
	{
		pos_incre  += (pos_now - pos_pre);
	}
	
	return pos_incre;
}

