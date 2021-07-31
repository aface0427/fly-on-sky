#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "Ano_Math.h"
#include "Drv_TFMini_Plus.h"
#include "Drv_OpenMV.h"
#include "Ano_Scheduler.h"
#include "Drv_HWT101CT.h"

#define TF_EXPECT_DIST  100.0f //期望距离
#define NORMALIZE_DIST  500.0f //距离阈值
#define NORMALIZE_SPEED 20.0f  //xy方向速度阈值
#define NORMALIZE_SPEED_YAW 30.0f //yaw方向速度阈值

#define MAX_SPEED_XY 20           //最大输出速度
#define MAX_SPEED_Z 15
#define MAX_SPEED_YAW 10 //yaw轴最大输出角速度

#define NORMALIZE_MV_DIST_Y 80.0f
#define NORMALIZE_MV_DIST_Z 60.0f
#define NORMALIZE_MPU_ANGLE_YAW 200.0f //yaw轴角度归一化
#define NORMALIZE_MOLCIRCLE_DIST_XY 120.0f //摩尔环xy方向阈值

s16 THR_Val = 800;
s16 CTRL_SPD_Z = 10;
s16 out_speed = 0;
s16 out_speed_y = 0;
s16 out_speed_z = 0;


enum AxialDirection axialdirection;

//20ms执行一次
void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step; 
		static u8 mission_task = 0;
		static u16 _cnt = 0;
    //判断有遥控信号才执行
    if (rc_in.no_signal == 0)
    {
				
        //判断第6通道拨杆位置 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
								one_key_takeoff_f = 1;
//								mission_task = 1;
//								mission_step = 1;
							
								//user_flag.tfmini_ctl_flag = 1;
								//user_flag.opmv_ctl_flag = 1;
								user_flag.hwt101_ctl_flag = 1;
							
								user_flag.openmv_clr_flag = 0;
								user_flag.yaw_set_flag = 1;
            }
						
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
				
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                //Add_Send_Data(0x41, );
								//FC_Lock();
//								OneKey_Hold();
							
								user_flag.tfmini_ctl_flag = 0;
								user_flag.opmv_ctl_flag = 0;
								user_flag.hwt101_ctl_flag = 0;
							
								user_flag.openmv_clr_flag = 1;
								user_flag.yaw_set_flag = 0;
							
							  mission_task = 0;
								mission_step = 0;
								one_key_land_f = 1;
								_cnt = 0;
								
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
				
				if(mission_task){
						switch(mission_step){
							case 0:
									break;
							case 1:
									
								
									break;
							case 2: 
								
									break;
									
							case 3:
						
									break;
							case 4:
							
									break;
							case 5:

									break;
							default:
									break;
						}
				}
			}
    ////////////////////////////////////////////////////////////////////////
}

//pid计算函数
float PID_calculate( float dT_s,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,//积分误差限幅
										float inte_lim			//integration limit，积分限幅									
										 )
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);
	
//	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	

	
	pid_val->exp_d = (expect - pid_val->exp_old) *hz;
	
	if(pid_arg->fb_d_mode == 0)
	{
		pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
	}
	else
	{
		pid_val->fb_d = pid_val->fb_d_ex;
	}	
	differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);
	
	pid_val->err = (expect - feedback);	

	pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;//)*T;//+ differential/pid_arg->kp
	//pid_val->err_i += pid_arg->ki *(pid_val->err )*T;//)*T;//+ pid_arg->k_pre_d *pid_val->feedback_d
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	
	
	pid_val->out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
			+	differential
//	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
	return (pid_val->out);
}


/*
*@fn:			u8 TFMini_Track(void)
*@brief:	利用TFmini返回x轴方向的距离，实现x轴的轴向定位
*@para:		none
*@return:	1
*@comment:
*/
u8 TFMini_Track(void){
    //测距所得与期望距离进行归一化处理
	float fdb_distance = tfmini.Dist > NORMALIZE_DIST? 1.0f: tfmini.Dist / NORMALIZE_DIST;
	float exp_distance = TF_EXPECT_DIST / NORMALIZE_DIST;
    
	float fdb_speed = 0;
	float exp_speed = 0;
	s16 _out_speed = 0;
	//位置环pid计算
	PID_calculate(0.02, 0, exp_distance, fdb_distance, &PID_Distance_arg_x, &PID_Distance_val_x, 0, 0);
    //位置环pid输出作为速度环的期望速度输入
	exp_speed = PID_Distance_val_x.out * -1;
    
	_out_speed = exp_speed * NORMALIZE_SPEED;
	
    //输出速度限位
	if(_out_speed > MAX_SPEED_XY)
		out_speed = MAX_SPEED_XY;
	else if(_out_speed < -1 * MAX_SPEED_XY)
		out_speed = -1 * MAX_SPEED_XY;
	else 
		out_speed = _out_speed;
	
    //发送对应输出指令
	RealTimeSpeedControl(out_speed, Direction_x);
	
	return 1;
}

/*
*@fn:			u8 HWT101CT_TRACK(void)
*@brief:	利用MPU6050返回yaw轴偏移，实现yaw轴定位
*@para:		none
*@return:	1
*@comment:
*/
u8 HWT101CT_TRACK(void){
	//测距所得与期望距离进行归一化处理
	float fdb_angle = hwt101ct.yaw_angle / NORMALIZE_MPU_ANGLE_YAW;
	float exp_angle = 0;
    
	float fdb_speed = 0;
	float exp_speed = 0;
	s16 _out_speed = 0;
	
	//位置环pid计算
	PID_calculate(0.02, 0, exp_angle, fdb_angle, &PID_Distance_arg_yaw, &PID_Distance_val_yaw, 0, 0);
  //位置环pid输出作为速度环的期望速度输入
	exp_speed = PID_Distance_val_yaw.out;
    
  //角速度反馈值归一化
	if(hwt101ct.yaw_speed > NORMALIZE_SPEED_YAW)
		fdb_speed = 1.0f;
	else if(hwt101ct.yaw_speed < -1 * NORMALIZE_SPEED_YAW)
		fdb_speed = -1.0f;
	else
		fdb_speed = hwt101ct.yaw_speed / NORMALIZE_SPEED_YAW;
	
  //速度环pid计算
	PID_calculate(0.02, 0, exp_speed, fdb_speed, &PID_Speed_arg_yaw, &PID_Speed_val_yaw, 0, 0);
    
	_out_speed = PID_Speed_val_yaw.out * NORMALIZE_SPEED_YAW;
	
  //输出速度限位
	if(_out_speed > MAX_SPEED_YAW)
		out_speed = MAX_SPEED_YAW;
	else if(_out_speed < -1 * MAX_SPEED_YAW)
		out_speed = -1 * MAX_SPEED_YAW;
	else 
		out_speed = _out_speed;
	
    //发送对应输出指令
	RealTimeSpeedControl(out_speed, Direction_yaw);
	
	return 1;
}

/*
*@fn:			u8 OpenMV_Track(void)
*@brief:	利用OpenMV返回在yz平面摄像头中心距标签的距离，实现yz方向的定位
*@para:		none
*@return:	1
*@comment:
*/
u8 OpenMV_Track(void){
	//判断openmv返回值是否有效
	if(opmv.cb.is_invalid){
		out_speed_y = 0;
		out_speed_z = 0;
		RealTimeSpeedControl(out_speed_y, Direction_y);
		RealTimeSpeedControl(out_speed_z, Direction_z);
		return 0;
	}
		
	//根据openmv返回的数据作归一化处理
	//y方向
	float fdb_distance_y = opmv.at.pos_y / NORMALIZE_MV_DIST_Y; //NORMALIZE_MV_DIST_Y = 80.0f
	float exp_distance_y = 0;
	float fdb_speed_y = 0;
	float exp_speed_y = 0;
	s16 _out_speed_y = 0;
	//z方向
	float fdb_distance_z = opmv.at.pos_z / NORMALIZE_MV_DIST_Z; //NORMALIZE_MV_DIST_Z = 60.0f
	float exp_distance_z= 0;
	float fdb_speed_z = 0;
	float exp_speed_z = 0;
	s16 _out_speed_z = 0;
	
  //位置环pid计算
	PID_calculate(0.02, 0, exp_distance_y, fdb_distance_y, &PID_Distance_arg_y, &PID_Distance_val_y, 0, 0);
	PID_calculate(0.02, 0, exp_distance_z, fdb_distance_z, &PID_Distance_arg_z, &PID_Distance_val_z, 0, 0);
	//输出取反
	exp_speed_y = PID_Distance_val_y.out * -1;
	exp_speed_z = PID_Distance_val_z.out * -1;
	
	_out_speed_y = exp_speed_y * NORMALIZE_SPEED;
	_out_speed_z = exp_speed_z * NORMALIZE_SPEED;
	
  //输出速度限位
	//y方向
	if(_out_speed_y > MAX_SPEED_XY)
		out_speed_y = MAX_SPEED_XY;
	else if(_out_speed_y < -1 * MAX_SPEED_XY)
		out_speed_y = -1 * MAX_SPEED_XY;
	else 
		out_speed_y = _out_speed_y;
	//z方向
	if(_out_speed_z > MAX_SPEED_Z)
		out_speed_z = MAX_SPEED_Z;
	else if(_out_speed_z < -1 * MAX_SPEED_Z)
		out_speed_z = -1 * MAX_SPEED_Z;
	else 
		out_speed_z = _out_speed_z;
	
  //发送对应输出指令
	RealTimeSpeedControl(out_speed_y, Direction_y);
	RealTimeSpeedControl(out_speed_z, Direction_z);
	
	return 1;
}

/*
*@fn:			u8 OpenMV_Circle_Track(void)
*@brief:	利用OpenMV返回在xy平面摄像头中心距同心摩尔环中心的距离，实现xy方向的定位
*@para:		none
*@return:	1
*@comment:
*/
u8 OpenMV_Circle_Track(void){
	//判断openmv返回值是否有效
	if(opmv.mol.is_invalid){
		out_speed_y = 0;
		out_speed_z = 0;
		RealTimeSpeedControl(out_speed_y, Direction_y);
		RealTimeSpeedControl(out_speed_z, Direction_z);
		return 0;
	}
		
	//根据openmv返回的数据作归一化处理
	//y方向
	float fdb_distance_y = opmv.mol.pos_y / NORMALIZE_MOLCIRCLE_DIST_XY; //NORMALIZE_MV_DIST_Y = 80.0f
	float exp_distance_y = 0;
	float fdb_speed_y = 0;
	float exp_speed_y = 0;
	s16 _out_speed_y = 0;
	//z方向
	float fdb_distance_z = opmv.mol.pos_z / NORMALIZE_MOLCIRCLE_DIST_XY; //NORMALIZE_MV_DIST_Z = 60.0f
	float exp_distance_z= 0;
	float fdb_speed_z = 0;
	float exp_speed_z = 0;
	s16 _out_speed_z = 0;
	
  //位置环pid计算
	PID_calculate(0.02, 0, exp_distance_y, fdb_distance_y, &PID_Distance_arg_y, &PID_Distance_val_y, 0, 0);
	PID_calculate(0.02, 0, exp_distance_z, fdb_distance_z, &PID_Distance_arg_z, &PID_Distance_val_z, 0, 0);
	//输出取反
	exp_speed_y = PID_Distance_val_y.out * -1;
	exp_speed_z = PID_Distance_val_z.out * -1;
	
	_out_speed_y = exp_speed_y * NORMALIZE_SPEED;
	_out_speed_z = exp_speed_z * NORMALIZE_SPEED;
	
  //输出速度限位
	//y方向
	if(_out_speed_y > MAX_SPEED_XY)
		out_speed_y = MAX_SPEED_XY;
	else if(_out_speed_y < -1 * MAX_SPEED_XY)
		out_speed_y = -1 * MAX_SPEED_XY;
	else 
		out_speed_y = _out_speed_y;
	
	//z方向
	if(_out_speed_z > MAX_SPEED_Z)
		out_speed_z = MAX_SPEED_Z;
	else if(_out_speed_z < -1 * MAX_SPEED_Z)
		out_speed_z = -1 * MAX_SPEED_Z;
	else 
		out_speed_z = _out_speed_z;
	
  //发送对应输出指令
	RealTimeSpeedControl(out_speed_y, Direction_y);
	RealTimeSpeedControl(out_speed_z, Direction_z);
	
	return 1;
}

/*
*@fn:			u8 RealTimeSpeedControl(s16 velocity, u8 direction)
*@brief:	通过实时控制帧控制指定方向(x、y、z轴)的速度
*@para:		s16 velocity 平移速度, u8 direction 平移方向
*@return:	1
*@comment:
*/
u8 RealTimeSpeedControl(s16 velocity, u8 direction){
//	if(velocity > SAFE_SPEED)
//		velocity = SAFE_SPEED;
	
	switch(direction){
		case Direction_x:
			rt_tar.st_data.vel_x = velocity;
			break;
		case Direction_y:
			rt_tar.st_data.vel_y = velocity;
			break;
		case Direction_z:
			rt_tar.st_data.vel_z = velocity;
			break;
		case Direction_yaw:
			rt_tar.st_data.yaw_dps = velocity;
		default:
			break;
	}
	return 1;
}

/*
*@fn:			u8 RealTimeSpeedControlSend(s16 velocity, u8 direction)
*@brief:	通过实时控制帧控制指定方向(x、y、z轴)的速度，并发送控制指令
*@para:		s16 velocity 平移速度, u8 direction 平移方向
*@return:	1
*@comment:
*/
u8 RealTimeSpeedControlSend(s16 velocity, u8 direction){
	RealTimeSpeedControl(velocity, direction);
	dt.fun[0x41].WTS = 1;
	
	return 1;
}

/*
*@fn:			u8 RealTimeSpeedControl_Angle(s16 velocity, u8 direction, u16 degree)
*@brief:	通过实时控制帧控制指定平面任意角度的平移速度，并发送控制指令
*@para:		s16 velocity 平移速度, u8 direction 平移平面, u16 degree 平移角度(单位：°)
*@return:	1
*@comment:对于不同平面角度值的方向定义
					xy	以飞机为圆心，x轴正方向为起点，向y轴正方向移动为正
					yz	以飞机为圆心，y轴正方向为起点，向z轴正方向移动为正
					xz	以飞机为圆心，x轴正方向为起点，向z轴正方向移动为正
*/
u8 RealTimeSpeedControl_Angle(s16 velocity, u8 direction, u16 degree){
	if(direction == Direction_xy){
		RealTimeSpeedControl(velocity * my_cos(degree / 360 * 3.14159265), Direction_x);
		RealTimeSpeedControl(velocity * my_sin(degree / 360 * 3.14159265), Direction_y);
	}
	else if(direction == Direction_yz){
		RealTimeSpeedControl(velocity * my_cos(degree / 360 * 3.14159265), Direction_y);
		RealTimeSpeedControl(velocity * my_sin(degree / 360 * 3.14159265), Direction_z);
	}
	else if(direction == Direction_xz){
		RealTimeSpeedControl(velocity * my_cos(degree / 360 * 3.14159265), Direction_x);
		RealTimeSpeedControl(velocity * my_sin(degree / 360 * 3.14159265), Direction_z);
	}
	//dt.fun[0x41].WTS = 1;
	
	return 1;
}

/*
*@fn:			u8 GeneralPosCtl()
*@brief:	通用定位函数
*@para:		
*@return:	1
*@comment:
*/
s16 GeneralPosCtl(_user_exp_fdb_set exp_fdb, 		//输入输出的期望与反馈
									u8 direction, 								//控制方向
									_PID_arg_st distance_arg, 			//pid位置参数结构体
									_PID_val_st distance_val,			//pid位置数据结构体
									_user_threshold_set threshold,//归一化与输出阈值
									u8 isInvert											//输出取反
								)
{
	float fdb_distance = exp_fdb.fdb_distance / threshold.normalize_distance;
	float exp_distance = exp_fdb.exp_distance / threshold.normalize_distance;
	float _out = 0;
	s16 _out_speed = 0;
	
  //位置环pid计算
	PID_calculate(0.02, 0, exp_distance, fdb_distance, &distance_arg, &distance_val, 0, 0);
	
	//输出取反
	if(isInvert == 1)
		_out = distance_val.out * -1;
	else
		_out = distance_val.out;
	
  //输出速度限位
	_out_speed = _out * threshold.max_speed;
	_out_speed = UserNormalize(_out_speed, threshold.max_speed, -1 * threshold.max_speed);
	
  //发送对应输出指令
	RealTimeSpeedControl(_out_speed, direction);
	
	return _out_speed;
}

/*
*@fn:			float UserNormalize(float num, float max, float min)
*@brief:	阈值保护函数
*@para:		float num 数据	，	float max 最大阈值 ， float min 最小阈值
*@return:	1
*@comment:
*/
float UserNormalize(float num, float max, float min){
	if(num > max)
		return max;
	else if(num < min)
		return min;
	else
		return num;
}

