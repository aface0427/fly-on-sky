/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "Drv_OpenMV.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_TFMini_Plus.h"
#include "Drv_HWT101CT.h"
#include "User_Task.h"

extern _ano_of_st ano_of;
_user_flag_set user_flag = {0};
s16 dx, dy;

//低通滤波测试参数
s16 dis_fix_x, dis_fix_y;
s16 dis_x, dis_y;

/*xy速度环参数*/
_PID_arg_st PID_Speed_arg_x; 
_PID_arg_st PID_Speed_arg_y; 
/*xy速度环位置*/
_PID_val_st PID_Speed_val_x;
_PID_val_st	PID_Speed_val_y;

/*xy位置环参数*/
_PID_arg_st PID_Distance_arg_x;
_PID_arg_st PID_Distance_arg_y; 
/*xy位置环参数*/
_PID_val_st PID_Distance_val_x;
_PID_val_st	PID_Distance_val_y;

/*yaw速度环参数*/
_PID_arg_st PID_Speed_arg_yaw;
/*yaw速度环参数*/
_PID_val_st PID_Speed_val_yaw;

/*yaw位置环参数*/
_PID_arg_st PID_Distance_arg_yaw;
/*yaw位置环参数*/
_PID_val_st PID_Distance_val_yaw;

/*z速度环参数*/
_PID_arg_st PID_Speed_arg_z;
/*z速度环参数*/
_PID_val_st PID_Speed_val_z;

/*z位置环参数*/
_PID_arg_st PID_Distance_arg_z;
/*z位置环参数*/
_PID_val_st PID_Distance_val_z;

/*x期望*/
_user_exp_fdb_set user_exp_fdb_x;
/*x阈值*/
_user_threshold_set user_threshold_x;
/*x测试输出*/
s16 test_output_x;

/*绕杆x阈值*/
_user_threshold_set user_threshold_pole_x;
/*绕杆x测试输出*/
s16 test_output_pole_x;


/*y期望*/
_user_exp_fdb_set user_exp_fdb_y;
/*y阈值*/
_user_threshold_set user_threshold_y;
/*y测试输出*/
s16 test_output_y;

/*z期望*/
_user_exp_fdb_set user_exp_fdb_z;
/*z阈值*/
_user_threshold_set user_threshold_z;
/*z测试输出*/
s16 test_output_z;

/*yaw期望*/
_user_exp_fdb_set user_exp_fdb_yaw;
/*yaw阈值*/
_user_threshold_set user_threshold_yaw;
/*yaw测试输出*/
s16 test_output_yaw;

/*imu欧拉角数据*/
_user_eula_set user_eula;

s16 Position_now;
s16 Position_pre;
s16 Position_incre;
//////////////////////////////////////////////////////////////////////
//用户程序调度器
//////////////////////////////////////////////////////////////////////

static void Loop_1000Hz(void) //1ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_100Hz(void) //10ms执行一次
{
	
	
}

static void Loop_50Hz(void) //20ms执行一次
{
	//////////////////////////////////////////////////////////////////////
  OpenMV_Offline_Check(20);
  TFmini_Offline_Check(20);
	UserTask_OneKeyCmd();
	//////////////////////////////////////////////////////////////////////
	u8 _dt = 20;
	static s32 dis_dx, dis_dy;
	static s32 _dis_x, _dis_y;
	
	/*速度积分*/
	if(user_flag.of_dis_clear_cmd){
		dis_dx = 0;
		dis_dy = 0;
	}
	else{
		dis_dx += _dt * ano_of.of2_dx_fix;
		dis_dy += _dt * ano_of.of2_dy_fix;
	}
	
	/*低通滤波*/
	_dis_x += _dt * ano_of.of1_dx;
	_dis_y += _dt * ano_of.of1_dy;
	dis_x = _dis_x / 1000;
	dis_y = _dis_y / 1000;
	dis_fix_x += (dis_x - dis_fix_x) * 0.2;
	dis_fix_y += (dis_y - dis_fix_y) * 0.2;
	
	/*实际距离数据*/
	dx = dis_dx / 1000;
	dy = dis_dy / 1000;
	
	dt.fun[0xf1].WTS = 1;
	dt.fun[0xf2].WTS = 1;
	dt.fun[0xf3].WTS = 1;
//		/*数据发送*/
//		User_DT_Send(ano_of, dx, dy);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50ms执行一次
{
	/*********************************TFmini x轴定位*******************************************/
  if(user_flag.tfmini_ctl_flag){
		/*TFmini控制x轴*/
		user_exp_fdb_x.exp_distance = 100;
		user_exp_fdb_x.fdb_distance = tfmini.Dist;
		test_output_x = GeneralPosCtl(user_exp_fdb_x, Direction_x, PID_Distance_arg_x, PID_Distance_val_x, user_threshold_x, 1);
	}
	
	/*********************************OpenMV yz轴定位*******************************************/
	if(user_flag.opmv_ctl_flag){
		/*OpenMV控制yz*/
		if(opmv.at.is_invalid){
			user_exp_fdb_y.fdb_distance = 0;
			user_exp_fdb_z.fdb_distance = 0;
		}
		else{
			user_exp_fdb_y.fdb_distance = opmv.at.pos_y;
			user_exp_fdb_z.fdb_distance = opmv.at.pos_z;
		}
		
		user_exp_fdb_y.exp_distance = 0;
		test_output_y = GeneralPosCtl(user_exp_fdb_y, Direction_y, PID_Distance_arg_y, PID_Distance_val_y, user_threshold_y, 1);
		
		user_exp_fdb_z.exp_distance = 0;
		test_output_z = GeneralPosCtl(user_exp_fdb_z, Direction_z, PID_Distance_arg_z, PID_Distance_val_z, user_threshold_z, 1);
	}
	
	/*********************************HWT101 yaw轴定位*******************************************/
	if(user_flag.hwt101_ctl_flag){
		/*hwt101保证yaw轴平稳*/
		if(user_flag.yaw_set_flag){
			user_exp_fdb_yaw.exp_distance = hwt101ct.yaw_angle;
			user_flag.yaw_set_flag = 0;
		}
		
		if(hwt101ct.offline){
			user_exp_fdb_yaw.fdb_distance = user_exp_fdb_yaw.exp_distance;
		}
		else{
			/*过零点判断*/
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
	}
		
	/*********************************绕杆*******************************************/
	if(user_flag.pole_ctl_flag){
		user_exp_fdb_x.exp_distance = 30;
		user_exp_fdb_x.fdb_distance = opmv.pole.pos_x;
		test_output_x = GeneralPosCtl(user_exp_fdb_x, Direction_x, PID_Distance_arg_x, PID_Distance_val_x, user_threshold_pole_x, 1);
		
		user_exp_fdb_yaw.exp_distance = 0;
		user_exp_fdb_yaw.fdb_distance = opmv.pole.angle_yaw;
		test_output_yaw = GeneralPosCtl(user_exp_fdb_yaw, Direction_yaw, PID_Distance_arg_yaw, PID_Distance_val_yaw, user_threshold_yaw, 1);
		
		rt_tar.st_data.vel_y = 10;
	}
	
	/*********************************数据位清零*******************************************/
	if(user_flag.openmv_clr_flag){
		user_flag.openmv_clr_flag = 0;
		RealTimeSpeedControl(0, Direction_x);
		RealTimeSpeedControl(0, Direction_y);
		RealTimeSpeedControl(0, Direction_z);
		RealTimeSpeedControl(0, Direction_yaw);
		Position_incre = 0;
		Position_pre = 0;
		//RealTimeSpeedControlSend(0, Direction_yaw);
	}
	
	/*发送实时控制帧*/
		dt.fun[0x41].WTS = 1;

	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500ms执行一次
{
}

/*
*@fn:			void Init_PID(void)
*@brief:	PID参数初始化
*@para:		none
*@return:	none
*@comment:
*/
void Init_PID(void){
//x速度环
	PID_Speed_arg_x.kp = 0.5f;
	PID_Speed_arg_x.ki = 0;
	PID_Speed_arg_x.kd_ex = 0;
	PID_Speed_arg_x.fb_d_mode = 0;
	PID_Speed_arg_x.kd_fb = 0;
	PID_Speed_arg_x.k_ff = 0;
	
//x位置环
	PID_Distance_arg_x.kp = 10.0f;
	PID_Distance_arg_x.ki = 0;
	PID_Distance_arg_x.kd_ex = 0;
	PID_Distance_arg_x.fb_d_mode = 0;
	PID_Distance_arg_x.kd_fb = 0;
	PID_Distance_arg_x.k_ff = 0;
	
//y速度环
	PID_Speed_arg_y.kp = 0.5f;
	PID_Speed_arg_y.ki = 0;
	PID_Speed_arg_y.kd_ex = 0;
	PID_Speed_arg_y.fb_d_mode = 0;
	PID_Speed_arg_y.kd_fb = 0;
	PID_Speed_arg_y.k_ff = 0;
	
//y位置环
	PID_Distance_arg_y.kp = 3.0f;
	PID_Distance_arg_y.ki = 0;
	PID_Distance_arg_y.kd_ex = 0;
	PID_Distance_arg_y.fb_d_mode = 0;
	PID_Distance_arg_y.kd_fb = 0;
	PID_Distance_arg_y.k_ff = 0;	
	
//yaw速度环
	PID_Speed_arg_yaw.kp = 0.5f;
	PID_Speed_arg_yaw.ki = 0;
	PID_Speed_arg_yaw.kd_ex = 0;
	PID_Speed_arg_yaw.fb_d_mode = 0;
	PID_Speed_arg_yaw.kd_fb = 0;
	PID_Speed_arg_yaw.k_ff = 0;
	
//yaw位置环
	PID_Distance_arg_yaw.kp = 5.0f;
	PID_Distance_arg_yaw.ki = 0;
	PID_Distance_arg_yaw.kd_ex = 0;
	PID_Distance_arg_yaw.fb_d_mode = 0;
	PID_Distance_arg_yaw.kd_fb = 0;
	PID_Distance_arg_yaw.k_ff = 0;	
	
//z速度环
	PID_Speed_arg_z.kp = 0.5f;
	PID_Speed_arg_z.ki = 0;
	PID_Speed_arg_z.kd_ex = 0;
	PID_Speed_arg_z.fb_d_mode = 0;
	PID_Speed_arg_z.kd_fb = 0;
	PID_Speed_arg_z.k_ff = 0;
	
//z位置环
	PID_Distance_arg_z.kp = 1.0f;
	PID_Distance_arg_z.ki = 0;
	PID_Distance_arg_z.kd_ex = 0;
	PID_Distance_arg_z.fb_d_mode = 0;
	PID_Distance_arg_z.kd_fb = 0;
	PID_Distance_arg_z.k_ff = 0;	
}

/*
*@fn:			void Init_GeneralCtlArg(void)
*@brief:	通用控制参数初始化
*@para:		none
*@return:	none
*@comment:
*/
void Init_GeneralCtlArg(void){
	/*x*/
	user_threshold_x.max_speed = 20;
	user_threshold_x.normalize_distance = 500.0f;
	user_threshold_x.normalize_speed = 20.0f;
	
	/*x绕杆*/
	user_threshold_pole_x.max_speed = 10;
	user_threshold_pole_x.normalize_distance = 50.0f;
	user_threshold_pole_x.normalize_speed = 10.0f;
	
	/*y*/
	user_threshold_y.max_speed = 20;
	user_threshold_y.normalize_distance = 80.0f;
	user_threshold_y.normalize_speed = 20.0f;
	
	/*z*/
	user_threshold_z.max_speed = 15;
	user_threshold_z.normalize_distance = 40.0f;
	user_threshold_z.normalize_speed = 15.0f;
	
	/*yaw*/
	user_threshold_yaw.max_speed = 20;
	user_threshold_yaw.normalize_distance = 200.0f;
	user_threshold_yaw.normalize_speed = 20.0f;
}

//////////////////////////////////////////////////////////////////////
//调度器初始化
//////////////////////////////////////////////////////////////////////
//系统任务配置，创建不同执行频率的“线程”
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
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for (index = 0; index < TASK_NUM; index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	for (index = 0; index < TASK_NUM; index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = GetSysRunTimeMs();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
