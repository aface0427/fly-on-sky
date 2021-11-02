void fuck(){
	switch(mission_step){
		case 0:
			cnt = 0;
			break;
		
		case 1:
			/*飞控解锁 */
			 FC_Unlock();
		
			/*3s延时*/
			cnt += dT;
			if(cnt >= 3000){
				cnt = 0;
				mission_step += 1;
			}
			break;
			
		case 2:
			/*一键起飞*/
			mission_step += OneKey_Takeoff(130);
			break;
		
		case 3:
			/*开启yaw轴自稳*/
			user_flag.yaw_set_flag = 1;
			Position_incre = 0;
			Position_pre = 0;
			cnt+=dT;
			if(cnt>=6000)
			mission_step += 2;
			break;
		
		case 4:
			/*识别停机坪的摩尔环并悬停8s*/
			opmv.mode_cmd[1] = 4; //摩尔环识别模式
			HWT101PosCtl(0);
			
			/*当mv返回的模式为摩尔环时，开启定位函数，同时计时*/
			if(opmv.mode_sta == 4){
				OpMVPosCtl_Down(0, 0);
				cnt += dT;
			}
			else{
				DataClr();
			}
			
			if(cnt >= 800){
				cnt = 0;
				mission_step += 1;
				DataClr();
				user_flag.yaw_set_flag = 1;
			}
			break;
			
		case 5:
			/*升高到130cm*/
			//HWT101PosCtl(0);
			//RealTimeSpeedControl(10, Direction_x);
			OFAltCtl(160);
			if(ano_of.of_alt_cm > 140 && ano_of.of_alt_cm < 180){
				cnt += dT;
			}
			else{
				cnt = 0;
			}
			
			if(cnt >= 20000){
				cnt = 0;
				mission_step += 1;
				user_flag.yaw_set_flag = 1;
				DataClr();
			}
			break;
		
		case 6:
			HWT101PosCtl(0);
			/*以10cm/s速度向前移动10s*/
			//RealTimeSpeedControl(10, Direction_x);
			opmv.mode_cmd[1] = 5; //红色杆识别模式
			cnt += dT;
		
			/*没识别到目标*/
			if(cnt >= 10000){
				cnt = 0;
				mission_step += 4;
				DataClr();
			}
			
			/*识别到目标*/
			if(!opmv.pole.is_invalid){
				DataClr();
				mission_step = 8;
			}
			break;
			
		/*未识别到杆*/
		case 7:
			/*原地自旋直到识别到杆*/
			opmv.mode_cmd[1] = 5; //红色杆识别模式
			RealTimeSpeedControl(15, Direction_yaw);
			if(!opmv.mol.is_invalid){
				DataClr();
				mission_step += 1;
			}
			break;
		
		/*识别到杆*/
		case 8:
			/*记录开始时的yaw角度*/
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			pos_start = pos_incre;
			mission_step += 1;	
			break;
		
		case 9:
			opmv.mode_cmd[1] = 5; //红色杆识别模式
			if(opmv.mode_sta == 5)
				PolePosCtl(50, -10, 0);
			pos_now = hwt101ct.yaw_angle;
			pos_incre = ZeroPointCross(pos_now, pos_pre, pos_incre);
			pos_pre = pos_now;
			
			if(UserAbs(pos_incre - pos_start) > 360){
				mission_step += 1;
				pos_incre = 0;
				pos_pre = 0;
				DataClr();
				user_flag.yaw_set_flag = 1;
			}
			break;
		
		case 10:
			/*yaw轴自稳，以-10cm/s速度沿x轴后退3s*/
			//RealTimeSpeedControl(-10, Direction_x);
			HWT101PosCtl(0);
			
			cnt += dT;
			if(cnt > 3000){
				cnt = 0;
				DataClr();
				mission_step += 1;
			}
			break;
			
		case 11:
			OneKey_Land();
			mission_step = 0;
			break;
		default:
			break;
	}
	
	return 1;
}