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


//pid�����ṹ��
typedef struct
{
	u8 fb_d_mode;
	float kp;			 //����ϵ��
	float ki;			 //����ϵ��
	float kd_ex;		 	 //΢��ϵ��
	float kd_fb; //previous_d ΢������
//	float inc_hz;  //����ȫ΢�ֵ�ͨϵ��
//	float k_inc_d_norm; //Incomplete ����ȫ΢�� ��һ��0,1��
	float k_ff;		 //ǰ�� 

}_PID_arg_st;

//pid���ݽṹ��
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

/*xy�ٶȻ�����*/
extern _PID_arg_st PID_Speed_arg_x; 
extern _PID_arg_st PID_Speed_arg_y; 
/*xy�ٶȻ�λ��*/
extern _PID_val_st PID_Speed_val_x;
extern _PID_val_st	PID_Speed_val_y;

/*xyλ�û�����*/
extern _PID_arg_st PID_Distance_arg_x; 
extern _PID_arg_st PID_Distance_arg_y;
/*xyλ�û�����*/
extern _PID_val_st PID_Distance_val_x;
extern _PID_val_st	PID_Distance_val_y;

/*yaw�ٶȻ�����*/
extern _PID_arg_st PID_Speed_arg_yaw;
/*yaw�ٶȻ�����*/
extern _PID_val_st PID_Speed_val_yaw;

/*yawλ�û�����*/
extern _PID_arg_st PID_Distance_arg_yaw;
/*yawλ�û�����*/
extern _PID_val_st PID_Distance_val_yaw;

/*z�ٶȻ�����*/
extern _PID_arg_st PID_Speed_arg_z;
/*z�ٶȻ�����*/
extern _PID_val_st PID_Speed_val_z;

/*zλ�û�����*/
extern _PID_arg_st PID_Distance_arg_z;
/*zλ�û�����*/
extern _PID_val_st PID_Distance_val_z;

/*imuŷ��������*/
extern _user_eula_set user_eula;;

/*flag��*/
typedef struct
{
	u8 of_dis_clear_cmd; //���λ������
	u8 init_pid_flag; //pid��ʼ����־λ
	u8 tfmini_ctl_flag;
	u8 opmv_ctl_flag;
	u8 hwt101_ctl_flag;
	u8 openmv_clr_flag;
	u8 openmv_down_flag;
	u8 yaw_set_flag; //yaw��Ƕ�ȷ��λ
	u8 pole_ctl_flag; //�Ƹ�ģʽ
	u8 of_alt_ctl_flag; //��������
} _user_flag_set;

/*�����뷴��*/
typedef struct
{
	s16 exp_distance;
	s16 fdb_distance;
} _user_exp_fdb_set;

/*��ֵ����*/
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
extern s16 test_output_alt_z;
extern s16 Position_now;
extern s16 Position_pre;
extern s16 Position_incre;
extern u16 cnt;
extern u8 test1;
extern float test2,test3,test4,test5,test6,test7;
void Init_PID(void);
void Init_GeneralCtlArg(void);
void Scheduler_Setup(void);
void Scheduler_Run(void);
void angle_fix(void);
u8 taskset2(s16 dT);
u8 taskset3(s16 dT);
u8 realtask(s16 dT);
u8 TFMiniPosCtl(s16 expect);
u8 OpMVPosCtl(s16 expect1, s16 expect2);
u8 OpMVPosCtl_Down(s16 expect1, s16 expect2);
u8 OpMVPosCtl_pole(s16 expect1,s16 expect2);
u8 HWT101PosCtl(s16 expect);
u8 PolePosCtl(s16 exp_x, s16 exp_y, s16 exp_yaw);
u8 OFAltCtl(u16 expect);
u8 DataClr(void);
u8 TaskSet(s16 dT);
s16 ZeroPointCross(s16 pos_now, s16 pos_pre, s16 pos_incre);
s16 UserAbs(s16 num);

#endif

