#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "SysConfig.h"
#include "User_Task.h"
typedef struct
{
void(*task_func)(void);
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

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

/*flag集*/
typedef struct
{
	u8 of_dis_clear_cmd; //清空位移数据
	u8 init_pid_flag; //pid初始化标志位
	u8 tfmini_ctl_flag;
	u8 openmv_clr_flag;
} _user_flag_set;

extern _user_flag_set user_flag;

void Init_PID(void);
void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif

