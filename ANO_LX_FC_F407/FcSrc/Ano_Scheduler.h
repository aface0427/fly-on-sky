#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "SysConfig.h"

typedef struct
{
void(*task_func)(void);
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

//pid参数结构体
typedef struct
{
	u8 fb_d_mode;
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd_ex;		 	 //微分系数
	float kd_fb; //previous_d 微分先行
//	float inc_hz;  //不完全微分低通系数
//	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 

}_PID_arg_st;

//pid数据结构体
typedef struct
{
	float err;
	float exp_old;
	float feedback_old;
	
	float fb_d;
	float fb_d_ex;
	float exp_d;
//	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;

	float out;
}_PID_val_st;

typedef struct{
	s16 pitch;
	s16 yaw;
	s16 rol;
	s16 fusion_sta;
}_user_eula_set;

/*xy速度环参数*/
extern _PID_arg_st PID_Speed_arg_x; 
extern _PID_arg_st PID_Speed_arg_y; 
/*xy速度环位置*/
extern _PID_val_st PID_Speed_val_x;
extern _PID_val_st	PID_Speed_val_y;

/*xy位置环参数*/
extern _PID_arg_st PID_Distance_arg_x; 
extern _PID_arg_st PID_Distance_arg_y;
/*xy位置环参数*/
extern _PID_val_st PID_Distance_val_x;
extern _PID_val_st	PID_Distance_val_y;

/*yaw速度环参数*/
extern _PID_arg_st PID_Speed_arg_yaw;
/*yaw速度环参数*/
extern _PID_val_st PID_Speed_val_yaw;

/*yaw位置环参数*/
extern _PID_arg_st PID_Distance_arg_yaw;
/*yaw位置环参数*/
extern _PID_val_st PID_Distance_val_yaw;

/*z速度环参数*/
extern _PID_arg_st PID_Speed_arg_z;
/*z速度环参数*/
extern _PID_val_st PID_Speed_val_z;

/*z位置环参数*/
extern _PID_arg_st PID_Distance_arg_z;
/*z位置环参数*/
extern _PID_val_st PID_Distance_val_z;

/*imu欧拉角数据*/
extern _user_eula_set user_eula;;

/*flag集*/
typedef struct
{
	u8 of_dis_clear_cmd; //清空位移数据
	u8 init_pid_flag; //pid初始化标志位
	u8 tfmini_ctl_flag;
	u8 opmv_ctl_flag;
	u8 hwt101_ctl_flag;
	u8 openmv_clr_flag;
	u8 yaw_set_flag; //yaw轴角度确定位
	u8 pole_ctl_flag; //绕杆模式
	u8 of_alt_ctl_flag; //光流定高
} _user_flag_set;

/*期望与反馈*/
typedef struct
{
	s16 exp_distance;
	s16 fdb_distance;
} _user_exp_fdb_set;

/*阈值集合*/
typedef struct
{
	float normalize_distance;
	float normalize_speed;
	s16 max_speed;
} _user_threshold_set;


extern _user_flag_set user_flag;
extern s16 dx, dy;
extern s16 dis_fix_x, dis_fix_y;
extern s16 dis_x, dis_y;
extern s16 test_output_x;
extern s16 test_output_y;
extern s16 test_output_z;
extern s16 test_output_yaw;
extern s16 test_output_pole_x;

extern s16 Position_now;
extern s16 Position_pre;
extern s16 Position_incre;

void Init_PID(void);
void Init_GeneralCtlArg(void);
void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif

