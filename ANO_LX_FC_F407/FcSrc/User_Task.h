#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "stm32f4xx.h"
#include "Ano_Scheduler.h"

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define SAFE_SPEED 200

void UserTask_OneKeyCmd(void);
u8 Vertical_Up(u16 height_cm, u16 velocity_cms);
extern float PID_calculate( float dT_s,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,//积分误差限幅
										float inte_lim			//integration limit，积分限幅									
										 );

extern s16 out_speed;
extern s16 out_speed_y;
extern s16 out_speed_z;

enum AxialDirection{
	Direction_x = 1,
	Direction_y,
	Direction_z,
	Direction_yaw,
	Direction_xy,
	Direction_yz,
	Direction_xz
};
extern enum AxialDirection axialdirection;


u8 TFMini_Track(void);
u8 OpenMV_Track(void);
u8 OpenMV_Circle_Track(void);
u8 HWT101CT_TRACK(void);
float UserNormalize(float num, float max, float min);
s16 GeneralPosCtl(_user_exp_fdb_set exp_fdb, 		//输入输出的期望与反馈
									u8 direction, 								//控制方向
									_PID_arg_st speed_arg, 			//pid速度参数结构体
									_PID_val_st speed_val,			//pid速度数据结构体
									_user_threshold_set threshold,//归一化与输出阈值
									u8 invert											//输出取反(1或-1)
								);
	
u8 RealTimeSpeedControl(s16 velocity, u8 direction);
u8 RealTimeSpeedControlSend(s16 velocity, u8 direction);
u8 RealTimeSpeedControl_Angle(s16 velocity, u8 direction, u16 degree);

#endif
