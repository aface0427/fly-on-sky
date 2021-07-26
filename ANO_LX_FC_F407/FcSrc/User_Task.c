#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "Ano_Math.h"

s16 THR_Val = 800;
s16 CTRL_SPD_Z = 10;

enum AxialDirection{
	Direction_x = 1,
	Direction_y,
	Direction_z,
	Direction_yaw,
	Direction_xy,
	Direction_yz,
	Direction_xz
}axialDirect;

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
								mission_task = 1;
								mission_step = 1;
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
								
									/*实时帧控制*/
										rt_tar.st_data.vel_x = 25;
										dt.fun[0x41].WTS = 1;
										mission_step += 1;
									
									/*命令帧控制*/
									//mission_step += Horizontal_Move(40, 20, 0); //以20cm/s的速度向前移动40cm
									
									//mission_step += FC_Unlock(); //解锁
									//LX_Change_Mode(3);
									break;
							case 2: 
									/*延时10s*/
									
									if(_cnt < 2000){
											if(_cnt % 200);
											else{
												rt_tar.st_data.vel_x = 25;
												dt.fun[0x41].WTS = 1;
											}
											_cnt += 20;
									}
									else{
											_cnt = 0;
											rt_tar.st_data.vel_x = 0;
											dt.fun[0x41].WTS = 1;
											mission_step += 1;
											//mission_step += OneKey_Takeoff(40);
									}
									break;
									
							case 3:
									/*延时10s*/
									if(_cnt < 3000)
											_cnt += 20;
									else{
											_cnt = 0;
											OneKey_Land();
											mission_step = 0;
											/*上升20cm，速度10cm/s*/
											//mission_step += Vertical_Up(10, 10);
											
											
											//mission_step += 1;
									}
									break;
							case 4:
									/*延时2s*/
									
//									if(_cnt < 100)
//											_cnt++;
//									else{
//											_cnt = 0;
//									}
//									mission_step += 1;
									break;
							case 5:
									mission_step = 0;
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

u8 AprilTag_Track(){
	

	
}

/*
*@fn:			u8 RealTimeSpeedControl(u16 velocity, u8 direction)
*@brief:	通过实时控制帧控制指定方向(x、y、z轴)的速度
*@para:		u16 velocity 平移速度, u8 direction 平移方向
*@return:	1
*@comment:
*/
u8 RealTimeSpeedControl(u16 velocity, u8 direction){
	if(velocity > SAFE_SPEED)
		velocity = SAFE_SPEED;
	
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
		default:
			break;
	}
	return 1;
}

/*
*@fn:			u8 RealTimeSpeedControlSend(u16 velocity, u8 direction)
*@brief:	通过实时控制帧控制指定方向(x、y、z轴)的速度，并发送控制指令
*@para:		u16 velocity 平移速度, u8 direction 平移方向
*@return:	1
*@comment:
*/
u8 RealTimeSpeedControlSend(u16 velocity, u8 direction){
	RealTimeSpeedControl(velocity, direction);
	dt.fun[0x41].WTS = 1;
	
	return 1;
}

/*
*@fn:			u8 RealTimeSpeedControl_Angle(u16 velocity, u8 direction, u16 degree)
*@brief:	通过实时控制帧控制指定平面任意角度的平移速度，并发送控制指令
*@para:		u16 velocity 平移速度, u8 direction 平移平面, u16 degree 平移角度(单位：°)
*@return:	1
*@comment:对于不同平面角度值的方向定义
					xy	以飞机为圆心，x轴正方向为起点，向y轴正方向移动为正
					yz	以飞机为圆心，y轴正方向为起点，向z轴正方向移动为正
					xz	以飞机为圆心，x轴正方向为起点，向z轴正方向移动为正
*/
u8 RealTimeSpeedControl_Angle(u16 velocity, u8 direction, u16 degree){
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
	dt.fun[0x41].WTS = 1;
	
	return 1;
}
