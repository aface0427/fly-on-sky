#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "stm32f4xx.h"

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define SAFE_SPEED 200

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

void UserTask_OneKeyCmd(void);
u8 Vertical_Up(u16 height_cm, u16 velocity_cms);
float PID_calculate( float dT_s,            //周期（单位：秒）
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
u8 MPU6050_Tack(void);

u8 RealTimeSpeedControl(s16 velocity, u8 direction);
u8 RealTimeSpeedControlSend(s16 velocity, u8 direction);
u8 RealTimeSpeedControl_Angle(s16 velocity, u8 direction, u16 degree);

#endif
