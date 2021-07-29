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
u8 MPU6050_Tack(void);
float UserNormalize(float num, float max, float min);
u8 GeneralPosCtl(_user_exp_fdb_set exp_fdb, 		//输入输出的期望与反馈
									u8 direction, 								//控制方向
									_PID_arg_st speed_arg, 			//pid速度参数结构体
									_PID_arg_st distance_arg,		//pid位置参数结构体
									_PID_val_st speed_val,			//pid速度数据结构体
									_PID_val_st distance_val,		//pid位置数据结构体
									_user_threshold_set threshold,//归一化与输出阈值
									s16 * output,									//用于上位机输出
									u8 invert											//输出取反(1或-1)
								);
	
u8 RealTimeSpeedControl(s16 velocity, u8 direction);
u8 RealTimeSpeedControlSend(s16 velocity, u8 direction);
u8 RealTimeSpeedControl_Angle(s16 velocity, u8 direction, u16 degree);

#endif
