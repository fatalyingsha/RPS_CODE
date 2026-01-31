	#include "balance_task.h"

	Balance_chassis_t b_chassis = {0};
	float V_T_gain;
	float V_Tp_gain;
	float balance_Tgain;
	float balance_Tpgain;

	float V_T_outlandgain;
	float V_Tp_outlandgain;
	float balance_Toutlandgain;
	float balance_Tpoutlandgain;

	float vw_torque = 0;
	float roll_F_output_angle = 0;
	float roll_leg_F_output = 0;
	uint8_t iout_flag;
	uint8_t midium_leg_flag;
	int16_t middle_cnt;
	int16_t seperate_cnt;
	int16_t down_cnt;
	u8 if_middle_leg;
	u8 if_low_middle_leg;
	u8 init_finish_flag;
	u8 down_flag;
	u8 down_cnt_flag;
	u8 over_slope_state=0;

	uint8_t allow_gimbal_init;
	 uint8_t test_flag_stand=0;
	u8 init_abnormal_state;


	/**
	************************************************************************************************************************
	* @Name     : balance_param_init
	* @brief    : 平衡底盘参数初始化
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float kp;
	float ki;
	float kd;
	uint32_t kmax=10000;
void balance_param_init(void)
{
			// 初始化底盘结构体，将其所有成员设置为0
			memset(&b_chassis, 0, sizeof(Balance_chassis_t));
			// 设置底盘的动态参考腿长
			b_chassis.chassis_dynemic_ref.leglength = 0.39;
		

			/*********************************正常姿态初始化Tp PID**********************************************************/
			PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 600, 200, 73, 0, 8);      // 初始化初始化Tp的PID//500, 200, 40, 0, 60    200

			// 初始化左腿的长度PID参数  50 0 700
		//	PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 180, 0, 50);
	//		PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 1200, 8, 30	);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
	//		PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 1200,8, 30);  //25 0 7

			/***************************************************************************************************/
	    PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 50, 0, 30	);
			PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 50, 0, 30	);
//
//			PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 4000, 0.01, 10	);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
//				PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 2000, 0, 200	);
//	PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 4000,0.01,10);  //25 0 7
//			PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 2000,0, 200);	
			b_chassis.chassis_ref.vy = 0; // 初始化速度为0
			b_chassis.chassis_ref.vx = 0;
			b_chassis.chassis_ref.vw = 0;

			/*********************************   双腿协调PID   ***************************************************/
			/********************************* ↓↓↓↓↓↓↓ ***************************************************/
			// 初始化双腿协调的PID控制器参数 2000, 2000, 150, 0, 3000  / 5, 3, 10, 0.001, 100
		//	PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 35, 3, 9.3, 0, 1.0); //速度环
		//	PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环//老车
		PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 35, 3, 9.3, 0, 1.0); //速度环
    PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环

			/*********************************  横滚角rollPID   ***************************************************/
			/********************************* ↓↓↓↓↓↓↓ ***************************************************/

			//小陀螺
			PID_struct_init(&b_chassis.roll_pid_angle, POSITION_PID, 5, 1, 0.2, 0, 0.8); //// 初始化roll轴滚角的PID参数  0.012, 0.00002, 0.02  30  2.2, 0, 1
			PID_struct_init(&b_chassis.roll_leg_F_Rotate_pid, POSITION_PID, 20, 10, 2, 0, 1);

			//普通模式：1, 1, 0.008, 0, 0.01； 20, 10, 2, 0, 1

			//普通模式
			PID_struct_init(&b_chassis.roll_leg_F_pid, POSITION_PID, 600, 10, 35, 0.01, 10);
			/********************************* 底盘旋转角速度PID   ***************************************************/
			/********************************* ↓↓↓↓↓↓↓ ***************************************************/
			PID_struct_init(&b_chassis.vw_pid, POSITION_PID, 10, 10, 3.5, 0.0, 2);     // 底盘旋转角速度W*底盘半径=底盘旋转线速度=vw 5, 5, 2, 0, 0

			/********************************* 底盘跟随云台PID   ***************************************************/
			/********************************* ↓↓↓↓↓↓↓ ***************************************************/
			PID_struct_init(&b_chassis.pid_follow_gim,POSITION_PID,3, 200, 8, 0, 1);// 初始化跟随云台的PID


			PID_struct_init(&b_chassis.pid_yaw_dist,POSITION_PID,10, 200, 5, 0, 0);

			/**********************************侧装甲板*********************************************/
			PID_struct_init(&b_chassis.pid_chassis_side,POSITION_PID,6, 200, 8, 0, 1);// 初始化跟随云台的PID

			/***********************************************************************************/
			PID_struct_init(&b_chassis.pid_seperate_gim,   POSITION_PID, 500, 200, 0.5, 0, 5); // 初始化分离云台的PID// 500, 200, 0.5, 0, 5

			PID_struct_init(&b_chassis.over_step_phi0_left_pid, POSITION_PID, 200, 200, 100, 0, 10);      // 初始化初始化Tp的PID//500, 200, 190, 0, 10    200
			PID_struct_init(&b_chassis.over_step_phi0_right_pid, POSITION_PID, 200, 200, 100, 0, 10);      // 初始化初始化Tp的PID//500, 200, 190, 0, 10    200

			/********************************* 异常姿态初始化处理PID   ***************************************************/
			/********************************* ↓↓↓↓↓↓↓ ***************************************************/
			PID_struct_init(&b_chassis.Init_Tp_dphi0_pid, POSITION_PID, 350, 200, 30, 0, 0);      // 初始化初始化Tp的PID//100, 200, 30, 0, 0
			PID_struct_init(&b_chassis.Init_Tp_phi0_pid, POSITION_PID,  1, 2, 1, 0, 0);  // 初始化初始化Tp的PID//500, 200, 40, 0, 60

			/*************************************************************************************************/
			PID_struct_init(&b_chassis.Init_dphi0_pid_left, POSITION_PID,  12, 2, 0.4, 0.001, 0);  // 初始化初始化Tp的PID//500, 200, 40, 0, 60
			PID_struct_init(&b_chassis.Init_dphi0_pid_right, POSITION_PID, 12, 2, 0.4, 0.001, 0); // 初始化初始化Tp的PID//500, 200, 40, 0, 60  腿很短时 3, 8, 0.8, 0, 0

			PID_struct_init(&b_chassis.Init_phi0_pid_left, POSITION_PID, 50, 200, 20, 0, 0);
			PID_struct_init(&b_chassis.Init_phi0_pid_right, POSITION_PID, 50, 200, 20, 0, 0);

//DaMiao_Motor_Enable(CAN1,0x01);
//DaMiao_Motor_Enable(CAN1,0x02);
//DaMiao_Motor_Enable(CAN1,0x03);
//DaMiao_Motor_Enable(CAN1,0x04);

}

	/**
	************************************************************************************************************************
	* @Name     : balance_chassis_task
	* @brief    : 平衡底盘总控制任务
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float target_angle;
	float ecd_speed;
	float lp_data = 0;
	float difference_left,difference_right,left_acc,right_acc;
	void stand(void)
	{ 	VMC_data_get(&b_chassis.left_leg,
							     DaMiao_8009_Joint_2.ecd.angle  * DM2_POLARITY,
									 DaMiao_8009_Joint_2.ecd.gyro   * DM2_POLARITY, 
									 DaMiao_8009_Joint_3.ecd.angle  * DM3_POLARITY,
									 DaMiao_8009_Joint_3.ecd.gyro   * DM3_POLARITY
									 );
		
		
						VMC_data_get(&b_chassis.right_leg, 
									 DaMiao_8009_Joint_1.ecd.angle * DM1_POLARITY ,
									 DaMiao_8009_Joint_1.ecd.gyro  * DM1_POLARITY,
									 DaMiao_8009_Joint_4.ecd.angle * DM4_POLARITY,
									 DaMiao_8009_Joint_4.ecd.gyro  * DM4_POLARITY
									 );
						uint8_t left_init_flag = 0, right_init_flag = 0;
					float left_leg_phi1 = normalize_angle_pi(b_chassis.left_leg.phi1);
			float right_leg_phi1 = normalize_angle_pi(b_chassis.right_leg.phi1);
			float phi0_0_2pi_left =transform_angle_0_2pi(b_chassis.left_leg.phi0-1.57f);
			float phi0_0_2pi_right=transform_angle_0_2pi(b_chassis.right_leg.phi0-1.57f);
			float phi0_0_2pi = (phi0_0_2pi_left+phi0_0_2pi_right)/2.0f;
	float  phi0_test_0_2pi,speed_ref;
	float  left_leg_phi1_test,right_leg_phi1_test;
			left_leg_phi1_test =left_leg_phi1;
			right_leg_phi1_test=right_leg_phi1;
			phi0_test_0_2pi=phi0_0_2pi_left;
				float dphi0= (DM4_POLARITY*b_chassis.right_leg.dphi0 + DM3_POLARITY*b_chassis.left_leg.dphi0)/2.0f;
			float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0
											float Init_Tp_dphi0 =pid_calc_angle_dir(&b_chassis.Init_Tp_phi0_pid, phi0_0_2pi,5.0,0);//顺时针
											float Init_Tp_out   = pid_calc(&b_chassis.Init_Tp_dphi0_pid,dphi0,Init_Tp_dphi0);

											// 双腿协调pid
											float  harmonize_outer = pid_calc(&b_chassis.leg_harmonize_pid_outer, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
											float  harmonize_inner = pid_calc(&b_chassis.leg_harmonize_pid_inner,(b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0), harmonize_outer);
											init_tp_calc(0.39,harmonize_inner,Init_Tp_out);
											// 电机输出限幅

											motor_torque_set(DM4_POLARITY * b_chassis.right_leg.T[1],DM3_POLARITY * b_chassis.left_leg.T[1],DM2_POLARITY * b_chassis.left_leg.T[0],DM1_POLARITY * b_chassis.right_leg.T[1],0,0);
					b_chassis.init_state=1;
			if(b_chassis.init_state==1)
			{ 
				float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0
				if ((fabs(phi0) >= 3 * PI / 180)&&fabs(b_chassis.left_leg.l0-b_chassis.right_leg.l0)<0.08)
				{
					float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid, phi0, 0);
									// 双腿协调pid
									float harmonize_outer = pid_calc(&b_chassis.leg_harmonize_pid_outer, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
									float harmonize_inner = pid_calc(&b_chassis.leg_harmonize_pid_inner, (b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0), harmonize_outer);
									init_tp_calc(0.10f,harmonize_inner,Init_Tp);
											motor_torque_set(DM4_POLARITY * b_chassis.right_leg.T[1],DM3_POLARITY * b_chassis.left_leg.T[1],DM2_POLARITY * b_chassis.left_leg.T[0],DM1_POLARITY * b_chassis.right_leg.T[1],0,0);
						
				}
			}
	}
	float FIRST_SET_ANGLE=0;
		void MODE_CHANGE_ONLY_CHASSIS(void)
		{
			
			b_chassis.last_ctrl_mode = b_chassis.ctrl_mode;
			if(RC_CtrlData.rc.ch3==1684)
			midium_leg_flag=1;
			else if(RC_CtrlData.rc.ch3==364)
					midium_leg_flag=0;
				
				if(RC_CtrlData.rc.s2==1)
				{
					b_chassis.jump_flag=1;
				}
								if(RC_CtrlData.rc.s2==3)
								{
				//					midium_leg_flag=0;
usart_chassis_data.chassis_mode=MANUAL_FOLLOW_REMOTE;
								}
			if(RC_CtrlData.rc.s2==2)
			{
usart_chassis_data.chassis_mode=CHASSIS_RELAX;
	b_chassis.ctrl_mode=CHASSIS_RELAX;


		
}			
//		if(        balance_chassis.Driving_Encoder[0].if_online
//								 && balance_chassis.Driving_En coder[1].if_online && time_tick > 2000
//					 )//  && gimbal_control_state_longtime==1
//			{
					// 如果控制模式不是初始化且不是待机模或获串口底盘数据底盘模式为零
					if (      (b_chassis.ctrl_mode != CHASSIS_INIT
										 && b_chassis.ctrl_mode != CHASSIS_STAND_MODE)
										|| usart_chassis_data.chassis_mode == 0)
					{
							// 更新盘控制模式设为串口数据控制底盘模式
							b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
					}
//					if(judge_rece_mesg.game_robot_state.power_management_chassis_output==0||judge_rece_mesg.game_robot_state.current_HP==0)
//					{
//							b_chassis.ctrl_mode=CHASSIS_RELAX;
//					}
					// 如果控制模式不是初始化
					if (b_chassis.ctrl_mode != CHASSIS_INIT)
					{
							// 更新roll轴的参考值
							b_chassis.chassis_ref.roll =usart_chassis_data.roll;
							// 更新y轴速度的参考值
							b_chassis.chassis_dynemic_ref.vy = usart_chassis_data.vy;
							// 更新x轴速度的参考值
							b_chassis.chassis_dynemic_ref.vx = usart_chassis_data.vx;
							//速度限幅
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy, b_chassis.min_speed, b_chassis.max_speed);
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx, -1.2, 1.2);
					}
					// 如果上一次的控制模式是空闲且当前控制模式不是初始化
					if (   b_chassis.ctrl_mode != CHASSIS_INIT
									&& usart_chassis_data.chassis_mode != CHASSIS_RELAX
									&& b_chassis.last_ctrl_mode == CHASSIS_RELAX
									&& usart_chassis_data.if_follow_gim==1)
					{
							// 更新控制模式
							b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
					}
					// 如果上一次的控制模式是空闲且当前控制模式不是空闲
					if (  b_chassis.last_ctrl_mode == CHASSIS_RELAX
									&& b_chassis.ctrl_mode != CHASSIS_RELAX)
					{
							DaMiao_Motor_Enable(CAN1,0x01);
	DaMiao_Motor_Enable(CAN1,0x02);
	DaMiao_Motor_Enable(CAN1,0x03);
	DaMiao_Motor_Enable(CAN1,0x04);
							// 更新控制模式为初始化
							b_chassis.ctrl_mode = CHASSIS_INIT;
					}
//					if ( ( (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode &&( fabs(chassis_gyro.pitch_Angle) > 15&&if_middle_leg!=1) && usart_chassis_data.chassis_mode  != CHASSIS_RELAX&&usart_chassis_data.ctrl_mode!=1)
//									|| (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode && ((b_chassis.balance_loop.theta> 0.71f)||((b_chassis.theta_left> 0.50f||b_chassis.theta_right>0.50f||fabs(chassis_gyro.pitch_Angle) > 40)&&if_middle_leg==1)|| (b_chassis.balance_loop.theta< -0.40f)) && usart_chassis_data.chassis_mode  != CHASSIS_RELAX&&usart_chassis_data.ctrl_mode!=1)
//									|| (( (fabs(chassis_gyro.pitch_Angle) > 40&&fabs(chassis_gyro.pitch_Angle)<100 )||fabs(chassis_gyro.roll_Angle)>95)&&usart_chassis_data.Rollover_posture_cmd==1) )
//									&&b_chassis.jump_flag!=1 // &&step_flag!=1////step_flag!=1//&& b_chassis.ctrl_mode == usart_chassis_data.chassis_mode//  || ((b_chassis.left_leg.phi0>0.7&&b_chassis.left_leg.phi0<1.3)&&(b_chassis.right_leg.phi0>0.7&&b_chassis.right_leg.phi0<1.3)
//						 )
//					{
//							// 更新控制模式为初始化
//							b_chassis.ctrl_mode = CHASSIS_INIT;
//					}
//					// 如果串口跳跃命令为1
//					if (usart_chassis_data.jump_cmd==1&&down_flag!=1)
//					{
//							// 跳跃标志位为1，等于串口跳跃命令
//							b_chassis.jump_flag = usart_chassis_data.jump_cmd;

//					}
//					if(usart_chassis_data.low_speed_cmd==1&&down_flag!=1)
//					{
//							b_chassis.jump_flag=usart_chassis_data.low_speed_cmd;
//							if_low_middle_leg =usart_chassis_data.low_speed_cmd;

//					}
if (if_middle_leg)
			{
					// 中间腿长计数器加一
					middle_cnt++;

			}
			// 如果中间腿长计数器等于1000
			if (middle_cnt ==1000||init_finish_flag==1)
			{
					// 中间腿长模式标志位为0
					if_middle_leg = 0;
					if_low_middle_leg=0;
					// 清除计数器
					middle_cnt = 0;

			}
//					if(b_chassis.chassis_dynemic_ref.vx != 0||b_chassis.ctrl_mode == CHASSIS_REVERSE)//b_chassis.chassis_dynemic_ref.vx != 0||TF02_state==0
//					{
//							b_chassis.jump_flag = 0;
//							if_low_middle_leg=0;
//							if_middle_leg=0;
//							//step_flag=0;

//					}
////					b_chassis.chassis_dynemic_ref.leglength = usart_chassis_data.cmd_leg_length;
//			}
//			else
//			{
//					// 否则更新控制模式为空闲
//					b_chassis.ctrl_mode = CHASSIS_RELAX;
//			}


			get_remote_angle();
			if (b_chassis.chassis_dynemic_ref.vx == 0) // 如果x轴期望速度为0
			{
					VAL_LIMIT(b_chassis.chassis_ref.remote_speed, b_chassis.min_speed, b_chassis.max_speed); // 速度限幅
			}
			else
			{
					VAL_LIMIT(b_chassis.chassis_ref.remote_speed, -1.5, 1.5);
			}
			
		}
		void MODE_CHANGE(void)
		{

//
							b_chassis.chassis_dynemic_ref.vy = usart_chassis_data.vy;

							b_chassis.chassis_dynemic_ref.vx = usart_chassis_data.vx;
							//速度限幅
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy, -1, 1);//b_chassis.min_speed，接受数据而改变，暂取1
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx, -1.2, 1.2);

					if(RC_CtrlData.rc.s2==3)
						b_chassis.ctrl_mode =MANUAL_FOLLOW_REMOTE;
			if(RC_CtrlData.rc.s2==2)
{b_chassis.ctrl_mode=CHASSIS_RELAX;
	b_chassis.last_ctrl_mode=CHASSIS_RELAX;
}			

b_chassis.last_ctrl_mode=b_chassis.ctrl_mode;
		}
 		void chassis_test_all(void)
		{
	
			 lp_data = Lpf_1st_calcu(&ACC_LPF, chassis_gyro.y_Acc, 10, 0.001); // 加速度计低通滤波
			//卡尔曼用一阶二阶做预测，三阶段输出
			if(fabs(b_chassis.chassis_ref.vw)>0.8)//0.75
			{
				//转角大时
					Mileage_kalman_filter_calc(&Mileage_kalman_filter,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].gyro + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].gyro) / 2.0f) * WHEEL_R,
																		 0);
			}
			else
			{
					Mileage_kalman_filter_calc(&Mileage_kalman_filter,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].gyro + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].gyro) / 2.0f) * WHEEL_R,
																		 chassis_gyro.y_Acc);
			}
			//一阶微分
			difference_left = difference_left_calc(balance_chassis.Driving_Encoder[0].gyro,0.001);
			difference_right =difference_right_calc(balance_chassis.Driving_Encoder[1].gyro,0.001);
			left_acc  = left_acc* 0.60f + difference_left * 0.40f;
			right_acc = right_acc*0.60f + difference_right* 0.40f;
			if (time_tick % 2 == 0)
			{

				MODE_CHANGE_ONLY_CHASSIS();
		//				MODE_CHANGE();//单模式测试
			
			switch(b_chassis.ctrl_mode)
			{
				case CHASSIS_RELAX:// 底盘空闲模式
					{
							// 置零所有关节的输出//////////////////////////
							b_chassis.joint_T[0] = 0;
							b_chassis.joint_T[1] = 0;
							b_chassis.joint_T[2] = 0;
							b_chassis.joint_T[3] = 0;
							b_chassis.driving_T[0] = 0;
							b_chassis.driving_T[1] = 0;
						b_chassis.left_leg.leglengthpid_outer.iout=0;
						b_chassis.right_leg.leglengthpid_outer.iout=0;
							//////////////////////////////////////////////////

							// 置零Tp/////////////////////////////////////
							balance_Tpgain = 0;
							balance_Tpoutlandgain = 0;
							b_chassis.chassis_ref.pitch = 0;
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
							b_chassis.normal_Y_erroffset = NORMAL_Y_ERROFFSET;
							b_chassis.roll_pid_angle.iout = 0;
							b_chassis.chassis_ref.roll = 0;
							///////////////////////////////////////////////////

							//置零标志位///////////////////////////////////
							b_chassis.init_state  = 0;
							b_chassis.rotate_flag = 0;
							b_chassis.jump_flag = 0;
							jump_state = 0;
							over_slope_state=0;
							jump_feedfront=0;

							if_low_middle_leg=0;
				  		if_middle_leg=0;

							init_abnormal_state=0;
							init_finish_flag=0;
							step_flag=0;
							step_state=0;
							midium_leg_flag=0;
							state_flag_2=0;
							state_flag_1=0;
							usart_chassis_data.fn_2_trigger_flag=0;
							//////////////////////////////////////////////

							b_chassis.left_leg.leg_FN = 100;
							b_chassis.right_leg.leg_FN = 100;
							//////////////////////////////////////////////
							
					}
					break;
				case CHASSIS_INIT:
				{
	
					chassis_Init_handle();
							
				}
				break;
					case CHASSIS_STAND_MODE: // 底盘起立模式
					{
							chassis_standup_handle(); // 起立模式
							balance_task();           // 平衡底盘解算
					}
					break;
					
				case MANUAL_FOLLOW_REMOTE:
				
			{
				if (b_chassis.jump_flag==1)
							{
							
						//		jump_task();
								balance_jump_handle(); 							
						//		chassis_follow_gim_handle();
								balance_task();
							}
							else
							{
	//							jump_state=0;
			chassis_follow_gim_handle();
		  balance_task();
							}
        
			}
			break;
				case CHASSIS_SEPARATE:
				{				
				}
					break;
						case CHASSIS_ROTATE: // 小陀螺
					{
							chassis_rotate_handle(); // 小陀螺模式
							balance_task();
					}
					break;
						case READYTORELAX:
				{
					if(b_chassis.ctrl_mode!=CHASSIS_RELAX)
					{
						
						
							b_chassis.joint_T[0]=trackRamp_leg(0.025,b_chassis.joint_T[0],0);
					b_chassis.joint_T[1]=trackRamp_leg(0.025,b_chassis.joint_T[1],0);
					b_chassis.joint_T[2]=trackRamp_leg(0.025,b_chassis.joint_T[2],0);
					b_chassis.joint_T[3]=trackRamp_leg(0.025,b_chassis.joint_T[3],0);
						

					}	
					
					
					
				
		
				}
				break;
				default:
					
					break;
			

			}
		
		}
	}
			
		void chassis_task(void)
	{
			  lp_data = Lpf_1st_calcu(&ACC_LPF, chassis_gyro.y_Acc, 10, 0.001); // 加速度计低通滤波
			if(fabs(b_chassis.chassis_ref.vw)>0.8)//0.75
			{
					Mileage_kalman_filter_calc(&Mileage_kalman_filter,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].gyro + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].gyro) / 2.0f) * WHEEL_R,
																		 0);
			}
			else
			{
					Mileage_kalman_filter_calc(&Mileage_kalman_filter,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R,
																		 ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].gyro + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].gyro) / 2.0f) * WHEEL_R,
																		 chassis_gyro.y_Acc);
			}
			difference_left = difference_left_calc(balance_chassis.Driving_Encoder[0].gyro,0.001);
			difference_right =difference_right_calc(balance_chassis.Driving_Encoder[1].gyro,0.001);
			left_acc  = left_acc* 0.60f + difference_left * 0.40f;
			right_acc = right_acc*0.60f + difference_right* 0.40f;

			//测试用（读取阈值）/////////////////////////////

	 //    chassis_Init_handle(); // 初始化收腿

			////////////////////////////////////////////////

			if (time_tick % 2 == 0)
			{
					chassis_cmd_selest();
					switch (b_chassis.ctrl_mode) // 状态机选择底盘控制模式
					{
					case CHASSIS_RELAX: // 底盘空闲模式
					{
							// 置零所有关节的输出//////////////////////////
							b_chassis.joint_T[0] = 0;
							b_chassis.joint_T[1] = 0;
							b_chassis.joint_T[2] = 0;
							b_chassis.joint_T[3] = 0;
							b_chassis.driving_T[0] = 0;
							b_chassis.driving_T[1] = 0;
							//////////////////////////////////////////////////

							// 置零Tp/////////////////////////////////////
							balance_Tpgain = 0;
							balance_Tpoutlandgain = 0;
							b_chassis.chassis_ref.pitch = 0;
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
							b_chassis.normal_Y_erroffset = NORMAL_Y_ERROFFSET;
							b_chassis.roll_pid_angle.iout = 0;
							b_chassis.chassis_ref.roll = 0;
							///////////////////////////////////////////////////

							//置零标志位///////////////////////////////////
							b_chassis.init_state  = 0;
							b_chassis.rotate_flag = 0;
							b_chassis.jump_flag = 0;
							jump_state = 0;
							over_slope_state=0;
							jump_feedfront=0;

							if_low_middle_leg=0;
							if_middle_leg=0;

							init_abnormal_state=0;

							step_flag=0;
							step_state=0;

							state_flag_2=0;
							state_flag_1=0;
							usart_chassis_data.fn_2_trigger_flag=0;
							//////////////////////////////////////////////

							b_chassis.left_leg.leg_FN = 100;
							b_chassis.right_leg.leg_FN = 100;
							//////////////////////////////////////////////
					}
					break;
					case CHASSIS_STOP: // 底盘停止模式
					{
							chassis_stop_handle();
							balance_task(); // 平衡底盘解算
					}
					break;
					case CHASSIS_INIT: // 底盘初始化模式
					{
							if(usart_chassis_data.fn_2_trigger_flag==1)
							{
									chassis_single_leg_handle();
							}
							else
									chassis_Init_handle(); // 初始化收腿
					}
					break;
					case CHASSIS_STAND_MODE: // 底盘起立模式
					{
							chassis_standup_handle(); // 起立模式
							balance_task();           // 平衡底盘解算
					}
					break;
					case MANUAL_FOLLOW_REMOTE: //
					{
							if (b_chassis.jump_flag==1) // 如果跳跃标志位为1//if (b_chassis.jump_flag==1)
							{
									over_slope_state=0;
									balance_jump_handle(); // 跳跃任务
									balance_task();        // 平衡底盘解算任务
							}

							else if(usart_chassis_data.overstep_cmd==1)
							{
									balance_over_slope_handle();
									balance_task();
							}
							else if(down_flag==1)
							{
									balance_down_run_handle();
							}
							else // 否则
							{
									over_slope_state=0;
									chassis_follow_gim_handle();
									balance_task();
							}
					}
					break;
					case CHASSIS_SEPARATE:
					{
							chassis_seperate_handle();
							balance_task();
					}
					break;
					case CHASSIS_ROTATE: // 小陀螺
					{
							chassis_rotate_handle(); // 小陀螺模式
							balance_task();
					}
					break;
					case CHASSIS_REVERSE:
					{
							chassis_side_handle();
							balance_task();
					}
					break;
					default:
							break;
					}
					if(usart_chassis_data.overstep_cmd==1)
					{
							b_chassis.max_speed = 2.0;

					}
					else
					{
							if(judge_rece_mesg.game_robot_state.robot_level<=2)
							{
									if(can_capacitance_message.cap_voltage_filte>20)
									{ 
											b_chassis.max_speed = 2.0;
									}
									else
											b_chassis.max_speed = 3.3;  // 最大速度
							}
							else
							{
									if(can_capacitance_message.cap_voltage_filte<6)
									{
											b_chassis.max_speed = 1.8;  // 最大速度//最小飞坡

									}
									else
											b_chassis.max_speed = 3.3;  // 最大速度
							}
					}
						 b_chassis.min_speed = -1.4; // 最小速度

			}
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_cmd_select
	* @brief    : 平衡底盘模式与命令处理
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float theta_test;
	void chassis_cmd_selest(void)
	{
			b_chassis.last_ctrl_mode = b_chassis.ctrl_mode;
			if(        balance_chassis.Driving_Encoder[0].if_online
								 && balance_chassis.Driving_Encoder[1].if_online && time_tick > 2000
					 )//  && gimbal_control_state_longtime==1
			{
					// 如果控制模式不是初始化且不是待机模或获串口底盘数据底盘模式为零
					if (      (b_chassis.ctrl_mode != CHASSIS_INIT
										 && b_chassis.ctrl_mode != CHASSIS_STAND_MODE)
										|| usart_chassis_data.chassis_mode == 0)
					{
							// 更新盘控制模式设为串口数据控制底盘模式
							b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
					}
					if(judge_rece_mesg.game_robot_state.power_management_chassis_output==0||judge_rece_mesg.game_robot_state.current_HP==0)
					{
							b_chassis.ctrl_mode=CHASSIS_RELAX;
					}
					// 如果控制模式不是初始化
					if (b_chassis.ctrl_mode != CHASSIS_INIT)
					{
							// 更新roll轴的参考值
							b_chassis.chassis_ref.roll =usart_chassis_data.roll;
							// 更新y轴速度的参考值
							b_chassis.chassis_dynemic_ref.vy = usart_chassis_data.vy;
							// 更新x轴速度的参考值
							b_chassis.chassis_dynemic_ref.vx = usart_chassis_data.vx;
							//速度限幅
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy, b_chassis.min_speed, b_chassis.max_speed);
							VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx, -1.2, 1.2);
					}
					// 如果上一次的控制模式是空闲且当前控制模式不是初始化
					if (   b_chassis.ctrl_mode != CHASSIS_INIT
									&& usart_chassis_data.chassis_mode != CHASSIS_RELAX
									&& b_chassis.last_ctrl_mode == CHASSIS_RELAX
									&& usart_chassis_data.if_follow_gim==1)
					{
							// 更新控制模式
							b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
					}
					// 如果上一次的控制模式是空闲且当前控制模式不是空闲
					if (  b_chassis.last_ctrl_mode == CHASSIS_RELAX
									&& b_chassis.ctrl_mode != CHASSIS_RELAX)
					{
							// 更新控制模式为初始化
							b_chassis.ctrl_mode = CHASSIS_INIT;
					}
					if ( ( (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode &&( fabs(chassis_gyro.pitch_Angle) > 15&&if_middle_leg!=1) && usart_chassis_data.chassis_mode  != CHASSIS_RELAX&&usart_chassis_data.ctrl_mode!=1)
									|| (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode && ((b_chassis.balance_loop.theta> 0.71f)||((b_chassis.theta_left> 0.50f||b_chassis.theta_right>0.50f||fabs(chassis_gyro.pitch_Angle) > 40)&&if_middle_leg==1)|| (b_chassis.balance_loop.theta< -0.40f)) && usart_chassis_data.chassis_mode  != CHASSIS_RELAX&&usart_chassis_data.ctrl_mode!=1)
									|| (( (fabs(chassis_gyro.pitch_Angle) > 40&&fabs(chassis_gyro.pitch_Angle)<100 )||fabs(chassis_gyro.roll_Angle)>95)&&usart_chassis_data.Rollover_posture_cmd==1) )
									&&b_chassis.jump_flag!=1 // &&step_flag!=1////step_flag!=1//&& b_chassis.ctrl_mode == usart_chassis_data.chassis_mode//  || ((b_chassis.left_leg.phi0>0.7&&b_chassis.left_leg.phi0<1.3)&&(b_chassis.right_leg.phi0>0.7&&b_chassis.right_leg.phi0<1.3)
						 )
					{
							// 更新控制模式为初始化
							b_chassis.ctrl_mode = CHASSIS_INIT;
					}
					// 如果串口跳跃命令为1
//					if (usart_chassis_data.jump_cmd==1&&down_flag!=1)
//					{
//							// 跳跃标志位为1，等于串口跳跃命令
//							b_chassis.jump_flag = usart_chassis_data.jump_cmd;

//					}
//					if(usart_chassis_data.low_speed_cmd==1&&down_flag!=1)
//					{
//							b_chassis.jump_flag=usart_chassis_data.low_speed_cmd;
//							if_low_middle_leg =usart_chassis_data.low_speed_cmd;

//					}

//					if(b_chassis.chassis_dynemic_ref.vx != 0||b_chassis.ctrl_mode == CHASSIS_REVERSE)//b_chassis.chassis_dynemic_ref.vx != 0||TF02_state==0
//					{
//							b_chassis.jump_flag = 0;
//							if_low_middle_leg=0;
//							if_middle_leg=0;
//							//step_flag=0;

//					}
//					b_chassis.chassis_dynemic_ref.leglength = usart_chassis_data.cmd_leg_length;
//			}
//			else
//			{
//					// 否则更新控制模式为空闲
//					b_chassis.ctrl_mode = CHASSIS_RELAX;
//			}
//			// 中间腿长模式逻辑
//			if (if_middle_leg)
//			{
//					// 中间腿长计数器加一
//					middle_cnt++;

//			}
//			// 如果中间腿长计数器等于1000
//			if (middle_cnt ==1000||init_finish_flag==1)
//			{
//					// 中间腿长模式标志位为0
//					if_middle_leg = 0;
//					if_low_middle_leg=0;
//					// 清除计数器
//					middle_cnt = 0;

//			}

//			if(b_chassis.ctrl_mode != CHASSIS_SEPARATE)
//			{
//					seperate_cnt = 0;

//			}
//			if(usart_chassis_data.ctrl_mode==1)
//			{
//					down_flag=1;
//					down_cnt_flag=1;
//			}
//			else
//			{
//	//        down_run_finish_flag=1;
//	//        if(can_capacitance_message.cap_voltage_filte > 4&&down_cnt > 1000)
//	//        {
//					down_flag = 0;
//					down_cnt_flag = 0;
//					down_cnt = 0;
//					//  }
//			}
//	//    if(down_flag==1)
//	//    {
//	//        b_chassis.ctrl_mode=CHASSIS_DOWN_MODE;
//	//    }

//			if(down_cnt_flag==1)
//			{
//					down_cnt++;
//			}
//			if(b_chassis.ctrl_mode != CHASSIS_ROTATE)
//			{
//					iout_flag = 0;
//			}
//			get_remote_angle();
//			if (b_chassis.chassis_dynemic_ref.vx == 0) // 如果x轴期望速度为0
//			{
//					VAL_LIMIT(b_chassis.chassis_ref.remote_speed, b_chassis.min_speed, b_chassis.max_speed); // 速度限幅
//			}
//			else
//			{
//					VAL_LIMIT(b_chassis.chassis_ref.remote_speed, -1.5, 1.5);
//			}
}
	}
	/**
	************************************************************************************************************************
	* @Name     : chassis_standup_handle
	* @brief    : 起立模式
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	void chassis_standup_handle(void)
	{
			PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 35, 3, 9.3, 0, 1.0); //速度环
			PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环

			b_chassis.chassis_ref.leglength = 0.12f; // 期望腿长给0.14
			b_chassis.chassis_ref.vy = 0;
			b_chassis.chassis_ref.vx = 0;
			b_chassis.chassis_ref.vw = 0;
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			if (fabs(b_chassis.balance_loop.state_err[4]) < 5 * PI / 180.0) // 如果phi角度误差小于2度
			{
						b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;  
							}
//			b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;      // 将控制模式设为串口底盘模式
	}

	/**
	************************************************************************************************************************
	* @Name     : chassis_Init_handle
	* @brief    : 初始化收腿
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float target_angle;
	float ecd_speed;
	float vw_angle;

	float  phi0_test_0_2pi,speed_ref;
	float  left_leg_phi1_test,right_leg_phi1_test;
	float phi0_test;
	uint16_t init_cnt;
	u8 state_flag_1=0,state_flag_2=0;
	float  Driving_T_left,Driving_T_right;

	void chassis_Init_handle(void)
	{

			jump_state=0;
		jump_feedfront = 0;
			b_chassis.jump_flag = 0;

			PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 600, 200, 73, 0, 180);      // 初始化初始化Tp的PID//500, 200, 40, 0, 60    200


//	PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 4000,0.01,10);  //25 0 7
//	PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 2000,0, 200);
//
//	PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 4000, 20000, 4000,0.01,10);  //25 0 7
//	PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 4000, 20000, 2000,0, 200);

		PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 4000, 0, 4800, 0,60000);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 4000, 0, 4800,0,60000);  //25 0 7

	

			
			b_chassis.chassis_ref.vy = 0; // 初始化速度为0
			b_chassis.chassis_ref.vx = 0;
			b_chassis.chassis_ref.vw = 0;
			 		VMC_data_get(&b_chassis.left_leg,
							     DaMiao_8009_Joint_2.ecd.angle  * DM2_POLARITY,
									 DaMiao_8009_Joint_2.ecd.gyro   * DM2_POLARITY, 
									 DaMiao_8009_Joint_3.ecd.angle  * DM3_POLARITY,
									 DaMiao_8009_Joint_3.ecd.gyro   * DM3_POLARITY
									 );
		
		
						VMC_data_get(&b_chassis.right_leg, 
									 DaMiao_8009_Joint_1.ecd.angle * DM1_POLARITY ,
									 DaMiao_8009_Joint_1.ecd.gyro  * DM1_POLARITY,
									 DaMiao_8009_Joint_4.ecd.angle * DM4_POLARITY,
									 DaMiao_8009_Joint_4.ecd.gyro  * DM4_POLARITY
									 );

			float left_leg_phi1 = normalize_angle_pi(b_chassis.left_leg.phi1);
			float right_leg_phi1 = normalize_angle_pi(b_chassis.right_leg.phi1);
			float phi0_0_2pi_left =transform_angle_0_2pi(b_chassis.left_leg.phi0-1.57f);
			float phi0_0_2pi_right=transform_angle_0_2pi(b_chassis.right_leg.phi0-1.57f);
			float phi0_0_2pi = (phi0_0_2pi_left+phi0_0_2pi_right)/2.0f;
			FN_Spring_left(&b_chassis.left_leg);
			FN_Spring_right(&b_chassis.right_leg);
			float left_sin_spring=cosf(b_chassis.left_leg.phi1-1.57)*b_chassis.left_leg.spring_FN;
			float right_sin_spring=cosf(b_chassis.right_leg.phi1-1.57)*b_chassis.right_leg.spring_FN;
			left_leg_phi1_test =left_leg_phi1;
			right_leg_phi1_test=right_leg_phi1;
			phi0_test_0_2pi=phi0_0_2pi_left;

			b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f) - chassis_gyro.pitch_Angle * PI / 180.0f);
			b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0) / 2.0f;
			//对dphi0出现NUN情况的处理
			if (isnan(b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0))
			{
					b_chassis.right_leg.dphi0 = 0.0f;
					b_chassis.left_leg.dphi0 = 0.0f;
			}
			float dphi0= (DM4_POLARITY*b_chassis.right_leg.dphi0 + DM3_POLARITY*b_chassis.left_leg.dphi0)/2.0f;
			float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0希望phi0在0附近，
			phi0_test=phi0;
			       if ((fabs(phi0) >= 4 * PI / 180)
						 &&(phi0_0_2pi_left<1.7||phi0_0_2pi_left>5.4)
						 &&(phi0_0_2pi_right<1.7||phi0_0_2pi_right>5.4)
						 &&chassis_gyro.pitch_Angle>-30
						 &&chassis_gyro.pitch_Angle<30)
        {
            b_chassis.chassis_ref.leglength = 0.10f; // 期望腿长给0.2
            float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid, phi0, 0);
            // 双腿协调pid
          float  harmonize_outer = pid_calc(&b_chassis.leg_harmonize_pid_outer, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
          float  harmonize_inner = pid_calc(&b_chassis.leg_harmonize_pid_inner, (b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0), harmonize_outer);
            // 腿部竖直力F的计算
            b_chassis.left_leg.leg_length_outer = pid_calc(&b_chassis.left_leg.leglengthpid_outer, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength);
            b_chassis.right_leg.leg_length_outer = pid_calc(&b_chassis.right_leg.leglengthpid_outer, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength);
					//   b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid_inner, b_chassis.left_leg.dl0, b_chassis.left_leg.leg_length_outer);
							b_chassis.left_leg.leg_F  =  b_chassis.left_leg.leg_length_outer+b_chassis.left_leg.spring_FN ;//加个角度
							b_chassis.right_leg.leg_F =  b_chassis.right_leg.leg_length_outer+b_chassis.right_leg.spring_FN ;
	
       //     b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid_inner, b_chassis.right_leg.dl0, b_chassis.right_leg.leg_length_outer);

            leg_conv(b_chassis.left_leg.leg_F, (Init_Tp - harmonize_inner) / 2.0f, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
            b_chassis.joint_T[1] = DM3_POLARITY *b_chassis.left_leg.T[0];
            b_chassis.joint_T[2] = DM2_POLARITY *b_chassis.left_leg.T[1];
            b_chassis.driving_T[0] = 0;

            leg_conv(b_chassis.right_leg.leg_F, (Init_Tp + harmonize_inner) / 2.0f, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
            b_chassis.joint_T[0] = DM4_POLARITY * b_chassis.right_leg.T[0];
            b_chassis.joint_T[3] = DM1_POLARITY * b_chassis.right_leg.T[1];
            b_chassis.driving_T[1] = 0;

            // 电机输出限幅
            limit_outer();
        }
        else if(chassis_gyro.pitch_Angle<-45||chassis_gyro.pitch_Angle>45)//翻车situation
		{
			
		}
		else if(phi0_0_2pi_left>1.7||phi0_0_2pi_right>1.7)//正坐地上
		{
			if(phi0_0_2pi_left>1.6)//左腿在其他情况
			{

					float Init_dphi0_Tp = pid_calc(&b_chassis.Init_dphi0_pid_left, DaMiao_8009_Joint_3.rate_rpm, -50);
									b_chassis.joint_T[1] = Init_dphi0_Tp;
									b_chassis.joint_T[2] = 0;
									b_chassis.driving_T[0] = 0;
			
			}
			if(phi0_0_2pi_right>1.6)
			{

					float Init_dphi0_Tp = pid_calc(&b_chassis.Init_dphi0_pid_right, DaMiao_8009_Joint_4.rate_rpm, +50);
									b_chassis.joint_T[0] = Init_dphi0_Tp;
									b_chassis.joint_T[3] = 0;
									b_chassis.driving_T[1] = 0;
			}
				
		}
				
				else
        {
            b_chassis.ctrl_mode = CHASSIS_STAND_MODE;
				}
	

	
}
	void limit_outer(void)
	{
			VAL_LIMIT(b_chassis.joint_T[1], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.joint_T[2], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.driving_T[0], -WHEEL_MAX_T, WHEEL_MAX_T);

			VAL_LIMIT(b_chassis.joint_T[0], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.joint_T[3], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.driving_T[1], -WHEEL_MAX_T, WHEEL_MAX_T);
	}

	void init_tp_calc(float ref_leglength,float harmonize,float init_Tp)
	{
			b_chassis.chassis_ref.leglength = ref_leglength; // 期望腿长给0.2
			// 腿部竖直力F的计算
			b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid_inner, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength);
			//b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid_inner, b_chassis.left_leg.dl0, b_chassis.left_leg.leg_length_outer);

			b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid_inner, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength);
			//b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid_inner, b_chassis.right_leg.dl0, b_chassis.right_leg.leg_length_outer);

			leg_conv(b_chassis.left_leg.leg_F, (init_Tp  - harmonize) / 2.0f, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
			leg_conv(b_chassis.right_leg.leg_F, (init_Tp + harmonize) / 2.0f, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);

	}
	void motor_torque_set(float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2)
	{
			//左
			b_chassis.joint_T[1] = Joint_T_1;//前
			b_chassis.joint_T[2] = Joint_T_2;
			b_chassis.driving_T[0] = Driving_T_1;
			//右
			b_chassis.joint_T[0] = Joint_T_0;//前
			b_chassis.joint_T[3] = Joint_T_3;
			b_chassis.driving_T[1] = Driving_T_2;
	}

	/**
	************************************************************************************************************************
	* @Name     : get_remote_angle
	* @brief    : 获取底盘转角
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float final_speed;
	float test_angle;
	void get_remote_angle(void)
	{
			float vy;
			float vx;
			float temp_angle;
			b_chassis.yaw_angle_0_2pi = chassis_gyro.yaw_Gyro;
			if (b_chassis.yaw_angle_0_2pi > PI) // 如果偏航角度大于π,将底盘的偏航角度转换为-π到π范围内
			{
					b_chassis.yaw_angle__pi_pi = b_chassis.yaw_angle_0_2pi - (2 * PI);
			} // 进行转换
			else
			{
					b_chassis.yaw_angle__pi_pi = b_chassis.yaw_angle_0_2pi;
			} // 否则直接赋值

			vy = b_chassis.chassis_dynemic_ref.vy; // 获取底盘的y轴速度
			vx = b_chassis.chassis_dynemic_ref.vx; // 获取底盘的x轴速度
			if (vy==0 && vx == 0)     // 如果y轴速度和x轴速度都为0
			{
					if(b_chassis.ctrl_mode==CHASSIS_REVERSE)
					{
							b_chassis.chassis_ref.remote_angle = PI/4;
							b_chassis.chassis_ref.remote_speed = 0;
					} else
					{
							// 否则将底盘的转角和速度设为0
							b_chassis.chassis_ref.remote_angle = 0;
							b_chassis.chassis_ref.remote_speed = 0;
					}
			}
			else
			{
					// 如果y轴速度和x轴速度不都为0
					b_chassis.chassis_ref.remote_speed = sqrt((vx * vx) + (vy * vy)); // 计算速度
					temp_angle = atan2(vy, vx) - 1.57f;                              // 计算角度
					if (temp_angle < -PI) // 如果角度小于-π，进行调整
					{
							b_chassis.chassis_ref.remote_angle = temp_angle + 2 * PI;
					}
					else // 否则直接赋值
					{
							b_chassis.chassis_ref.remote_angle = temp_angle;
					}
			}
	}


	/**
	************************************************************************************************************************
	* @Name     : chassis_follow_gim_handle
	* @brief    : 底盘跟随云台
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/

	void chassis_follow_gim_handle(void)
	{
		
		b_chassis.jump_flag=0;
		PID_struct_init(&b_chassis.roll_pid_angle, POSITION_PID, 3, 1, 0.002, 0, 0.01); //// 初始化roll轴滚角的PID参数  0.012, 0.00002, 0.02  30  2.2, 0, 1
			PID_struct_init(&b_chassis.roll_leg_F_Rotate_pid, POSITION_PID, 400, 10, 80 , 0,300);//大改，，
//			b_chassis.chassis_ref.roll = usart_chassis_data.roll;
	


PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 2000, 0, 2400, 0,40000);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 2000, 0, 2400,0,40000);  //25 0 7

	

	


			b_chassis.left_leg.leglengthpid_outer.iout=0;
	b_chassis.right_leg.leglengthpid_outer.iout=0;


get_remote_angle();
if(midium_leg_flag==1)
{b_chassis.chassis_dynemic_ref.leglength=0.19;}
else
{b_chassis.chassis_dynemic_ref.leglength=0.10;}
					b_chassis.chassis_ref.leglength = trackRamp_leg(0.001,b_chassis.chassis_ref.leglength,b_chassis.chassis_dynemic_ref.leglength);//b_chassis.chassis_dynemic_ref.leglength
	//		PID_struct_init(&b_chassis.pid_follow_gim,POSITION_PID,6, 200, 8, 0, 1);// 初始化跟随云台的PID 8, 200, 8, 0, 1
		init_abnormal_state=0;
		PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 35, 3, 9.3, 0, 1.0); //速度环
    PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 50); //角度环


			if (fabs(b_chassis.balance_loop.dx) > 0.8//0.5
							||b_chassis.chassis_ref.vy != 0
							||fabs(b_chassis.chassis_ref.vw) >= 2.5//1.75
							||usart_chassis_data.ctrl_mode==1)
			{
				
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			}		// 位置等于x，err为0
			else
			{
					b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.001 * TIME_STEP; // 否则加上err补偿
			}
//			if (fabs(b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi) < PI / 2) // 如果云台角度与底盘偏航角差值小于PI/2,在同一象限内
//			{
//					target_angle = b_chassis.chassis_ref.remote_angle;
//					ecd_speed = b_chassis.chassis_ref.remote_speed;
//					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
//			}
//			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 3 * PI / 2) // 如果云台角度与底盘偏航角差值大于3*PI/2
//			{
//					target_angle = b_chassis.chassis_ref.remote_angle - 2 * PI;
//					ecd_speed = b_chassis.chassis_ref.remote_speed;
//					b_chassis.chassis_ref.roll =  usart_chassis_data.roll;
//			}
//			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < -3 * PI / 2) // 如果差值的绝对值小于-3π/2
//			{
//					target_angle = b_chassis.chassis_ref.remote_angle + 2 * PI;
//					ecd_speed = b_chassis.chassis_ref.remote_speed;
//					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
//			}
//			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 0) // 如果差值的绝对值大于0
//			{
//					target_angle = b_chassis.chassis_ref.remote_angle - PI;
//					ecd_speed = -b_chassis.chassis_ref.remote_speed;
//					b_chassis.chassis_ref.roll =  -usart_chassis_data.roll;
//			}
//			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < 0) // 如果差值的绝对值小于0
//			{
//					target_angle = b_chassis.chassis_ref.remote_angle + PI;
//					ecd_speed = -b_chassis.chassis_ref.remote_speed;
//					b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
//			}
ecd_speed=(RC_CtrlData.rc.ch1-1024)/660.0f*b_chassis.max_speed;
target_angle =(RC_CtrlData.rc.ch2-1024)/660.0f*6;//其实是ref_vw
//b_chassis.chassis_dynemic_ref.leglength=(RC_CtrlData.rc.ch3-1024)/660.0f*0.39;
if(b_chassis.chassis_dynemic_ref.leglength<0.10)
{
	b_chassis.chassis_dynemic_ref.leglength=0.10;
}
			b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy, ecd_speed); // 将当前值调整到参考值（平滑移动

			b_chassis.chassis_ref.vw = trackRamp(b_chassis.chassis_ref.vw, target_angle);
//b_chassis.chassis_ref.vw =0;// - pid_calc(&b_chassis.pid_follow_gim,chassis_gyro.yaw_Angle, 0);
//			b_chassis.chassis_ref.vw = - pid_calc(&b_chassis.pid_follow_gim, b_chassis.yaw_angle__pi_pi, target_angle);


			if(b_chassis.chassis_dynemic_ref.leglength>0.24)
			{
					VAL_LIMIT(b_chassis.chassis_ref.vy, -1.9, 1.9);
			}
			if(fabs(b_chassis.chassis_ref.vw)>=2.0)
			{
					VAL_LIMIT(b_chassis.chassis_ref.vy, -1.0, 1.0);
			}


			if(b_chassis.rotate_flag==1)//
			{
					if(fabs(chassis_gyro.yaw_Gyro * PI / 180.0f) >= 0.5)//0.5
					{
							b_chassis.chassis_ref.vy =0;
					}
					else
					{
							b_chassis.rotate_flag=0;
					}
			}
	}

	/**
	************************************************************************************************************************
	* @Name     : chassis_single_leg_handle
	* @brief    : 底盘单腿控制
	* @param		: void
	* @retval   : void
	* @Note     :翻车干涉时用
	************************************************************************************************************************
	**/
	void chassis_single_leg_handle(void)
	{

			VMC_data_get(&b_chassis.left_leg, balance_chassis.joint_Encoder[2].angle * DM3_POLARITY,
									 balance_chassis.joint_Encoder[2].gyro * DM3_POLARITY,
									 balance_chassis.joint_Encoder[1].angle * DM2_POLARITY  + PI,
									 balance_chassis.joint_Encoder[1].gyro * DM2_POLARITY);

			VMC_data_get(&b_chassis.right_leg, balance_chassis.joint_Encoder[3].angle * DM4_POLARITY,
									 balance_chassis.joint_Encoder[3].gyro * DM4_POLARITY,
									 balance_chassis.joint_Encoder[0].angle * DM1_POLARITY + PI,
									 balance_chassis.joint_Encoder[0].gyro * DM1_POLARITY);
			float phi0_0_2pi_left =transform_angle_0_2pi(b_chassis.left_leg.phi0-1.57f);
			float phi0_0_2pi_right=transform_angle_0_2pi(b_chassis.right_leg.phi0-1.57f);

			float phi0_ref_left  =usart_chassis_data.leg_single_angle_handle_left +phi0_0_2pi_left;
			float phi0_ref_right =usart_chassis_data.leg_single_angle_handle_right+phi0_0_2pi_right;

			float Init_Tp_left  = pid_calc(&b_chassis.Init_phi0_pid_left,  phi0_0_2pi_left,  phi0_ref_left );
			float Init_Tp_right = pid_calc(&b_chassis.Init_phi0_pid_right, phi0_0_2pi_right, phi0_ref_right);

			leg_conv(0,  Init_Tp_left,  b_chassis.left_leg.phi1,  b_chassis.left_leg.phi4,  b_chassis.left_leg.T );
			leg_conv(0,  Init_Tp_right, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);

											motor_torque_set(DM4_POLARITY * b_chassis.right_leg.T[1],DM3_POLARITY * b_chassis.left_leg.T[1],DM2_POLARITY * b_chassis.left_leg.T[0],DM1_POLARITY * b_chassis.right_leg.T[1],0,0);
			limit_outer();


	}


	/**
	************************************************************************************************************************
	* @Name     : chassis_follow_gyro
	* @brief    : 底盘跟随陀螺仪
	* @param		: void
	* @retval   : void
	* @Note     :单底盘测试用
	************************************************************************************************************************
	**/
	void chassis_follow_gyro(void)
	{

			if(b_chassis.chassis_dynemic_ref.vy>0&&b_chassis.chassis_dynemic_ref.vy<0.04)
			{
					b_chassis.chassis_ref.vy=0;
			}
			else
			{
					b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
			}

			if(b_chassis.chassis_dynemic_ref.vx>0&&b_chassis.chassis_dynemic_ref.vx<0.02)
			{
					b_chassis.chassis_ref.vx=0;
			}
			else
					b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;

			b_chassis.chassis_ref.roll = 0.0f;
			if (if_middle_leg) // 如果中间腿长标志位为1
			{
					b_chassis.chassis_ref.leglength = 0.25f; // 设置腿长
			}
			else
					b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;

			if (fabs(b_chassis.chassis_ref.vx) > 0.06)
					b_chassis.chassis_ref.vw = b_chassis.chassis_ref.vx;

			if (fabs(b_chassis.balance_loop.dx) > 0.5 || fabs(b_chassis.chassis_ref.vy) > 0.15 || b_chassis.chassis_ref.vw >= 0.2) // 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
			{
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			} // 位置等于x，err为0
			else
			{

					b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.00001 * TIME_STEP;

			} // 否则加上err补偿
	}

	/**
	************************************************************************************************************************
	* @Name     : chassis_rotate_handle
	* @brief    : 小陀螺
	* @param	: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/

	void chassis_rotate_handle(void)
	{

			b_chassis.rotate_flag=1;	
		PID_struct_init(&b_chassis.roll_pid_angle, POSITION_PID, 10, 1, 0.021, 0, 0.01); //// 初始化roll轴滚角的PID参数  0.03, 0.0003, 0.01  ；20, 10, 3, 0, 1
//		PID_struct_init(&b_chassis.roll_leg_F_Rotate_pid, POSITION_PID, 20, 10, 2.96, 0, 1.0);
			
			VAL_LIMIT(b_chassis.roll_pid_angle.iout, -0.001, 0.001);              // 绕Z轴的角速度值限制在-15到15
get_remote_angle();
		
			if(b_chassis.yaw_angle_0_2pi>=PI)
			{
					b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);
			}
			else
			{
					b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;
			}
			// 腿长为b_chassis.chassis_dynemic_ref.leglength
			b_chassis.chassis_ref.leglength =0.13;
			//b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx; // 
			if(b_chassis.chassis_ref.vw>1)
			b_chassis.chassis_ref.vy=b_chassis.chassis_dynemic_ref.vy*cosf(b_chassis.yaw_angle__pi_pi-3*PI/4)+b_chassis.chassis_dynemic_ref.vx*sinf(b_chassis.yaw_angle__pi_pi+PI/4);
//		b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy*sinf(b_chassis.yaw_angle__pi_pi+PI/4)+b_chassis.chassis_dynemic_ref.vx*cosf(b_chassis.yaw_angle__pi_pi+PI/4);
			else
b_chassis.chassis_ref.vy=b_chassis.chassis_dynemic_ref.vy*cosf(b_chassis.yaw_angle__pi_pi+PI/2)+b_chassis.chassis_dynemic_ref.vx*sinf(b_chassis.yaw_angle__pi_pi-PI/2);
			
			b_chassis.chassis_ref.vx = 0;


			// 绕Z轴的角速度为USART接收的数据中的旋转速度
			b_chassis.chassis_ref.vw = 8;
			//b_chassis.chassis_ref.vw=3;
			//VAL_LIMIT(b_chassis.chassis_ref.vw, -15, 15);              // 绕Z轴的角速度值限制在-15到15
//			if(fabs(b_chassis.chassis_dynemic_ref.vy)+fabs(b_chassis.chassis_dynemic_ref.vx)>0.8)
//		VAL_LIMIT(b_chassis.chassis_ref.vw, -3, 3);   
			VAL_LIMIT(b_chassis.chassis_ref.vy, -5.0, 5.0);            // y轴方向的速度值限制在-0.5到0.5
	}
	/**
	************************************************************************************************************************
	* @Name     : balance_jump_handle
	* @brief    : 跳跃任务
	* @param	: jump_state
	* @retval   : void
	* @Note     : jump_state用于记录跳跃状态
	************************************************************************************************************************
	**/
	u8    jump_state = 0;
	float jump_feedfront = 0;
	u8    jump_vm_flag;
	float distance;

	void balance_jump_handle(void)
	{
			b_chassis.chassis_ref.vx = 0; // 将在X轴方向上的线速度和绕Z轴的角速度初始化为0。
			b_chassis.chassis_ref.vw = 0;
			init_finish_flag=0;

			if (jump_state == 0)
			{

					PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 45, 3, 11, 0, 1.0); //速度环
					PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环
	//				PID_struct_init(&b_chassis.pid_follow_gim,POSITION_PID,10, 200, 15, 0, 1);// 初始化跟随云台的PID


					jump_feedfront = 0;
					b_chassis.chassis_ref.leglength = 0.11f; // 设置腿长


					if (fabs(b_chassis.balance_loop.dx) > 0.5
									||b_chassis.chassis_ref.vy != 0
									||fabs(b_chassis.chassis_ref.vw) >= 3.0
									||usart_chassis_data.ctrl_mode==1)// 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
					{
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					} // 位置等于x，err为0
					else
					{

							b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

					} // 否则加上err补偿
//					if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
//					{
//							target_angle = b_chassis.chassis_ref.remote_angle;
//							ecd_speed = b_chassis.chassis_ref.remote_speed;
//							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
//					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
//					{
//							target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
//							ecd_speed = b_chassis.chassis_ref.remote_speed;
//							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
//					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
//					{
//							target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
//							ecd_speed = b_chassis.chassis_ref.remote_speed;
//							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
//					}
//					else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
//					{
//							target_angle = b_chassis.chassis_ref.remote_angle - PI;
//							ecd_speed = -b_chassis.chassis_ref.remote_speed;
//							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
//					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
//					{
//							target_angle = b_chassis.chassis_ref.remote_angle + PI;
//							ecd_speed = -b_chassis.chassis_ref.remote_speed;
//							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
//					}
ecd_speed=(RC_CtrlData.rc.ch1-1024)/660.0f*2.2;
target_angle =(RC_CtrlData.rc.ch2-1024)/660.0f*6;
				 b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy,ecd_speed);
				 			b_chassis.chassis_ref.vw = trackRamp(b_chassis.chassis_ref.vw, target_angle);
					VAL_LIMIT(b_chassis.chassis_ref.vy, -1.73, 1.73);//-1.5, 1.5  mea_distance<0.48

//					b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);  //这里不使用remoteangle，因为测距在前面
//				distance = (mea_distance)*cosf(b_chassis.balance_loop.phi)*cosf(b_chassis.yaw_angle__pi_pi);
					if (RC_CtrlData.rc.ch3==1024)//mea_distance<0.48//        if (mea_distance_left<0.49||mea_distance_right<0.49)//mea_distance<0.48//  0.515
					{
							jump_state++;
					}
			}
			else if (jump_state == 1)
			{
					jump_vm_flag=1;
					jump_feedfront = 460;                                                                               // 前馈
					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                                             // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                                      // 位置，直接等于x，err等于0
			//		PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 900, 0, 0);
					PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 15000, 0, 0);    //气弹簧参数    700, 0, 0    70, 0, 0  410, 0, 0 55, 0, 0

			//		PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 910, 0, 0);
					PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 15000, 0, 0);
				

					b_chassis.chassis_ref.leglength = 0.39f;                                                          // 腿长为0.38                                                          // 腿长为0.38

				
					if (RC_CtrlData.rc.ch3!=1684)//mea_distance<0.48//        if (mea_distance_left<0.49||mea_distance_right<0.49)//mea_distance<0.48//  0.515
					{
							jump_state++;
					}
				//	if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.015)&&(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.015)) // 如果腿长误差小于0.01
				//			jump_state++;                                                                                 // 跳跃状态+1
			}
			else if (jump_state == 2)
			{
					jump_feedfront = 0; // 前馈

					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                          // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                   // 位置，直接等于x，err等于0

					b_chassis.chassis_ref.leglength = 0.12f;                                       // 腿长
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
					{
							PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 35, 3, 9.3, 0, 1.0); //速度环
							PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环
							// 初始化左腿的长度PID参数  50 0 700
					//		PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 80, 0, 20);
				PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 15000,3,0	);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
			PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 15000,3,0);  //25 0 7  //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50
																																																				//25 0 7
							jump_state = 0;                                                                                   // 跳跃状态标志位归零
							b_chassis.jump_flag = 0;                                                                       // 跳跃任务标志位为0
							if_middle_leg = 1;
							// 中间腿标志位为1
					}
			}
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_jump_distance_yaw_handle
	* @brief    : 跳跃任务测试版-双测距yaw角闭环
	* @param	: jump_state_disance
	* @retval   : void
	* @Note     : jump_state_disance记录跳跃状态
	************************************************************************************************************************
	**/
	float estimated_chassis_yaw = 0.0f;    // 估算的底盘yaw角度
	float prev_gimbal_yaw = 0.0f;          // 上一时刻云台yaw角度
	float filtered_dist_error = 0.0f;      // 滤波后的测距误差
	float prev_dist_left=0.0f,prev_dist_right=0.0f,max_dist_jump=0.05;
	u8 trust_dist;
	void yaw_control_update(float mea_distance_left,float mea_distance_right)
	{
			// ---------- 1. 测距信任判断 ----------////
			float dl_jump = fabsf(mea_distance_left - prev_dist_left);
			float dr_jump = fabsf(mea_distance_right - prev_dist_right);
			float mean_dist = (mea_distance_left + mea_distance_right) / 2.0f;
			if (dl_jump > max_dist_jump || dr_jump > max_dist_jump) {
					trust_dist = 0;  // 如果差异过大，不信任测距数据
			} else {
					trust_dist = 1;   // 测距数据正常，信任
			}
			prev_dist_left = mea_distance_left;
			prev_dist_right = mea_distance_right;


			float raw_dist_error = (mea_distance_left-mean_dist) - (mea_distance_right-mean_dist);
			filtered_dist_error = 0.92 * filtered_dist_error +
														0.08 * raw_dist_error;
			float vw_from_dist = pid_calc(&b_chassis.pid_yaw_dist, filtered_dist_error, 0);
			float final_vw_ref = 0.0f;
			if (trust_dist) {
					final_vw_ref = 0.6f * vw_from_dist + 0.4f *  -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);
			} else {
					final_vw_ref = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);  //这里不使用remoteangle，因为测距在前面
			}


			b_chassis.chassis_ref.vw = final_vw_ref;

	}
	u8    jump_state_disance = 0;


	void balance_jump_distance_yaw_handle(void)
	{
			// yaw_control_update(mea_distance_left,mea_distance_right);

			//b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);  //这里不使用remoteangle，因为测距在前面


			b_chassis.chassis_ref.vx = 0; // 将在X轴方向上的线速度和绕Z轴的角速度初始化为0。
			init_finish_flag=0;

			if (jump_state_disance == 0)
			{

					jump_feedfront = 0;
					b_chassis.chassis_ref.leglength = 0.15f; // 设置腿长


					if (fabs(b_chassis.balance_loop.dx) > 0.5
									||b_chassis.chassis_ref.vy != 0
									||fabs(b_chassis.chassis_ref.vw) >= 3.0
									||usart_chassis_data.ctrl_mode==1)// 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
					{
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					} // 位置等于x，err为0
					else
					{

							b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

					} // 否则加上err补偿
					if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					}
					else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
					{
							target_angle = b_chassis.chassis_ref.remote_angle - PI;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
					{
							target_angle = b_chassis.chassis_ref.remote_angle + PI;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					}

					b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy,ecd_speed);

					VAL_LIMIT(b_chassis.chassis_ref.vy, -1.5, 1.5);//-1.4, 1.4  mea_distance<0.48
					if (mea_distance<0.49)//mea_distance<0.48
					{
							jump_state_disance++;
					}
			}
			else if (jump_state_disance == 1)
			{
					jump_vm_flag=1;
					jump_feedfront = 230;                                                                               // 前馈
					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                                             // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                                      // 位置，直接等于x，err等于0
					PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 860, 0, 0);
					PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 120, 0, 0);    //气弹簧参数    700, 0, 0    70, 0, 0  410, 0, 0 55, 0, 0

					PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 870, 0, 0);
					PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 110, 0, 0);

					b_chassis.chassis_ref.leglength = 0.37f;                                                          // 腿长为0.38
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
							jump_state_disance++;                                                                                 // 跳跃状态+1
			}
			else if (jump_state_disance == 2)
			{
					jump_feedfront = 0; // 前馈

					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                          // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                   // 位置，直接等于x，err等于0

					b_chassis.chassis_ref.leglength = 0.14f;                                       // 腿长
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
					{
							// 初始化左腿的长度PID参数  50 0 700
							PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 20);
							PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 30, 0, 0);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50

							/***************************************************************************************************/
							// 初始化右腿的长度PID参数  50 0 700
							PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 20);
							PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 30, 0, 0);  //25 0 7
							jump_state_disance = 0;                                                                                   // 跳跃状态标志位归零
							b_chassis.jump_flag = 0;                                                                          // 跳跃任务标志位为0
							if_middle_leg = 1;
							// 中间腿标志位为1
					}
			}
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_jump_double_handle
	* @brief    : 跳跃任务测试版-连续跳跃
	* @param	: jump_state_double
	* @retval   : void
	* @Note     : jump_state_double记录跳跃状态
	************************************************************************************************************************
	**/
	u8    jump_state_double = 0;
	uint16_t jump_cnt;
	void balance_jump_double_handle(void)
	{
			b_chassis.chassis_ref.vx = 0; // 将在X轴方向上的线速度和绕Z轴的角速度初始化为0。
			b_chassis.chassis_ref.vw = 0;
			init_finish_flag=0;
			if (jump_state_double == 0)
			{

					jump_feedfront = 0;
					b_chassis.chassis_ref.leglength = 0.15f; // 设置腿长


					if (fabs(b_chassis.balance_loop.dx) > 0.5
									||b_chassis.chassis_ref.vy != 0
									||fabs(b_chassis.chassis_ref.vw) >= 3.0
									||usart_chassis_data.ctrl_mode==1)// 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
					{
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					} // 位置等于x，err为0
					else
					{

							b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

					} // 否则加上err补偿
					if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
					{
							target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					}
					else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
					{
							target_angle = b_chassis.chassis_ref.remote_angle - PI;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
					{
							target_angle = b_chassis.chassis_ref.remote_angle + PI;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					}

					b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy,ecd_speed);

					VAL_LIMIT(b_chassis.chassis_ref.vy, -1.5, 1.5);//-1.4, 1.4  mea_distance<0.48

					b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);  //这里不使用remoteangle，因为测距在前面

					if (mea_distance<0.49)//mea_distance<0.48
					{
							jump_state_double++;
					}
			}
			else if (jump_state_double == 1)
			{
					jump_vm_flag=1;
					jump_feedfront = 230;                                                                               // 前馈
					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                                             // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                                      // 位置，直接等于x，err等于0
					PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 860, 0, 0);
					PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 120, 0, 0);    //气弹簧参数    700, 0, 0    70, 0, 0  410, 0, 0 55, 0, 0

					PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 870, 0, 0);
					PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 20000, 20000, 110, 0, 0);

					b_chassis.chassis_ref.leglength = 0.37f;                                                          // 腿长为0.38
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
							jump_state_double++;                                                                                 // 跳跃状态+1
			}
			else if (jump_state_double == 2)
			{
					jump_feedfront = 0; // 前馈

					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                          // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                   // 位置，直接等于x，err等于0

					b_chassis.chassis_ref.leglength = 0.14f;                                       // 腿长
					if((fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
					{


							jump_state_double++;

					}
			}
			else if(jump_state_double==3)
			{
					jump_feedfront = 230; // 前馈

					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                          // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                   // 位置，直接等于x，err等于0

					b_chassis.chassis_ref.leglength = 0.32f;                                       // 腿长
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
					{
							jump_state_double++;

					}
			}
			else if(jump_state_double==4)
			{
					jump_feedfront = 0; // 前馈

					b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;                          // x轴方向的速度
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;                   // 位置，直接等于x，err等于0

					b_chassis.chassis_ref.leglength = 0.15f;                                       // 腿长
					if( (fabs(b_chassis.left_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)||(fabs(b_chassis.right_leg.l0 - b_chassis.chassis_ref.leglength) <= 0.01)) // 如果腿长误差小于0.01
					{
							// 初始化左腿的长度PID参数  50 0 700
							PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 80, 0, 20);
							PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 30, 0, 0);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50

							/***************************************************************************************************/
							// 初始化右腿的长度PID参数  50 0 700
							PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 80, 0, 20);
							PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 30, 0, 0);  //25 0 7
							jump_state_double = 0;                                                                                   // 跳跃状态标志位归零
							b_chassis.jump_flag = 0;                                                                          // 跳跃任务标志位为0
							// if_middle_leg = 1;
							// 中间腿标志位为1

					}
			}
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_over_slope_handle
	* @brief    : 飞坡模式
	* @param	:
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	uint16_t over_slope_leglenth_cnt;
	void balance_over_slope_handle(void)
	{
			// 初始化左腿的长度PID参数  50 0 700
			PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 10);
			PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 5000, 20000, 25, 0, 0);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50

			/***************************************************************************************************/
			// 初始化右腿的长度PID参数  50 0 700
			PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 10);
			PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 5000, 20000, 25,  0, 0);  //25 0 7

			if(over_slope_state==0)
			{
					b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
					if(fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<0.02)
							over_slope_state++;
			}
			else
			{
					if (b_chassis.balance_loop.L0<0.165)
					{
							b_chassis.chassis_ref.leglength = 0.15;
							over_slope_leglenth_cnt++;
							if(over_slope_leglenth_cnt>2500)
							{
									over_slope_state=0;
									over_slope_leglenth_cnt=0;
							}
					}
					else
					{
							b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;

					}
			}
			if (fabs(b_chassis.balance_loop.dx) > 0.5
							||b_chassis.chassis_ref.vy != 0
							||fabs(b_chassis.chassis_ref.vw) >= 3.0
							||usart_chassis_data.ctrl_mode==1) // 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
			{
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			} // 位置等于x，err为0
			else
			{
					b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

			} // 否则加上err补偿
			if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
			{
					target_angle = b_chassis.chassis_ref.remote_angle;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
			{
					target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
			{
					target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			}
			else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
			{
					target_angle = b_chassis.chassis_ref.remote_angle - PI;
					ecd_speed = -b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
			} else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
			{
					target_angle = b_chassis.chassis_ref.remote_angle + PI;
					ecd_speed = -b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
			}

			b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy,ecd_speed);
			VAL_LIMIT(b_chassis.chassis_ref.vy, -2.0, 2.0);//1.9 1.8
			b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);

			if(b_chassis.rotate_flag==1)
			{
					if(fabs(chassis_gyro.yaw_Gyro * PI / 180.0f) >= 0.5)
					{
							b_chassis.chassis_ref.vy =0;
					}
					else
					{
							b_chassis.rotate_flag=0;
					}
			}
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_step_handle
	* @brief    : 上台阶模式
	* @param	: step_state
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	uint8_t step_state=0,step_flag=0;
	uint16_t step_state_cnt=0;
	uint8_t leg_state=0;
	float phi0,remain_phi0_right,ref_phi0_left,ref_phi0_right,ref_leglenth_left,ref_leglenth_right;
	void balance_step_handle(void)
	{
	//    // 初始化左腿的长度PID参数  50 0 700
	//    PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 10);
	//    PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 25, 0, 0);    //气弹簧参数 85, 0, 10  25, 0, 0 ， 95, 0, 50

	//    /***************************************************************************************************/
	//    // 初始化右腿的长度PID参数  50 0 700
	//    PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 85, 0, 10);
	//    PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 3000, 20000, 25,  0, 0);  //25 0 7

	//    b_chassis.chassis_ref.vx = 0; // 将在X轴方向上的线速度和绕Z轴的角速度初始化为0。
	//    b_chassis.chassis_ref.vw = 0;

	//    if(step_state==0)//站立
	//    {
	//        balance_task();
	//        float phi0_0_2pi_left =transform_angle_0_2pi(b_chassis.left_leg.phi0-1.57f);
	//        float phi0_0_2pi_right=transform_angle_0_2pi(b_chassis.right_leg.phi0-1.57f);

	//        b_chassis.chassis_ref.leglength = 0.36f;

	//        if (fabs(b_chassis.balance_loop.dx) > 0.5
	//                ||b_chassis.chassis_ref.vy != 0
	//                ||fabs(b_chassis.chassis_ref.vw) >= 3.0
	//                ||usart_chassis_data.ctrl_mode==1)// 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
	//        {
	//            b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
	//        } // 位置等于x，err为0
	//        else
	//        {
	//            b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

	//        } // 否则加上err补偿
	//        if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
	//        {
	//            target_angle = b_chassis.chassis_ref.remote_angle;
	//            ecd_speed = b_chassis.chassis_ref.remote_speed;
	//            b_chassis.chassis_ref.roll = usart_chassis_data.roll;
	//        } else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
	//        {
	//            target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
	//            ecd_speed = b_chassis.chassis_ref.remote_speed;
	//            b_chassis.chassis_ref.roll = usart_chassis_data.roll;
	//        } else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
	//        {
	//            target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
	//            ecd_speed = b_chassis.chassis_ref.remote_speed;
	//            b_chassis.chassis_ref.roll = usart_chassis_data.roll;
	//        }
	//        else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
	//        {
	//            target_angle = b_chassis.chassis_ref.remote_angle - PI;
	//            ecd_speed = -b_chassis.chassis_ref.remote_speed;
	//            b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
	//        } else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
	//        {
	//            target_angle = b_chassis.chassis_ref.remote_angle + PI;
	//            ecd_speed = -b_chassis.chassis_ref.remote_speed;
	//            b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
	//        }
	//        b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy,ecd_speed);

	//        VAL_LIMIT(b_chassis.chassis_ref.vy, -1.0, 1.0);//-1.4, 1.4  mea_distance<0.48

	//        b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0);

	//        if(b_chassis.balance_loop.theta>0.5)
	//        {
	//            remain_phi0_right=phi0_0_2pi_right;

	//            step_state_cnt++;

	//            if(step_state_cnt>500)
	//            {
	//                step_state++;
	//            }
	//        }
	//    }
	//    else if(step_state==1)//左腿先起，右腿再起
	//    {


	//        VMC_data_get(&b_chassis.left_leg, balance_chassis.joint_Encoder[2].angle * DM3_POLARITY,
	//                     balance_chassis.joint_Encoder[2].gyro * DM3_POLARITY,
	//                     balance_chassis.joint_Encoder[1].angle * DM2_POLARITY + PI,
	//                     balance_chassis.joint_Encoder[1].gyro * DM2_POLARITY);

	//        VMC_data_get(&b_chassis.right_leg, balance_chassis.joint_Encoder[3].angle * DM4_POLARITY,
	//                     balance_chassis.joint_Encoder[3].gyro * DM4_POLARITY,
	//                     balance_chassis.joint_Encoder[0].angle * DM1_POLARITY + PI,
	//                     balance_chassis.joint_Encoder[0].gyro * DM1_POLARITY);
	//        phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0

	//        float phi0_0_2pi_left =transform_angle_0_2pi(b_chassis.left_leg.phi0-1.57f);
	//        float phi0_0_2pi_right=transform_angle_0_2pi(b_chassis.right_leg.phi0-1.57f);
	//        //   float phi0_0_2pi = (phi0_0_2pi_left+phi0_0_2pi_right)/2.0f;

	//        b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f) - chassis_gyro.pitch_Angle * PI / 180.0f);
	//        if(leg_state==0)
	//        {

	//            ref_leglenth_left=0.36;
	//            ref_leglenth_right=0.36;
	//            ref_phi0_right=remain_phi0_right;
	//            ref_phi0_left=2.0;
	//            if(fabs(phi0_0_2pi_left-ref_phi0_left)<0.02)
	//            {
	//                leg_state=1;
	//            }
	//        }
	//        else if(leg_state==1)
	//        {
	//            ref_leglenth_left=0.14;
	//            ref_leglenth_right=0.36;
	//            ref_phi0_right=remain_phi0_right;
	//            ref_phi0_left=0.86;

	//            if(fabs(phi0_0_2pi_left-ref_phi0_left)<0.02)
	//            {
	//                leg_state=2;
	//            }
	//        }
	//        else if(leg_state==2)
	//        {

	//            ref_leglenth_left=0.14;
	//            ref_leglenth_right=0.36;
	//            ref_phi0_right=2.0;
	//            ref_phi0_left=0.86;
	//            if(fabs(phi0_0_2pi_right-ref_phi0_right)<0.02)
	//            {
	//                leg_state=3;
	//            }
	//        }
	//        else if(leg_state==3)
	//        {
	//            ref_leglenth_left=0.14;
	//            ref_leglenth_right=0.14;
	//            ref_phi0_right=0.86;
	//            ref_phi0_left=0.86;


	//        }

	//        float Init_Tp_phi0_left =pid_calc(&b_chassis.over_step_phi0_left_pid, phi0_0_2pi_left, ref_phi0_left);

	//        float Init_Tp_phi0_right=pid_calc(&b_chassis.over_step_phi0_right_pid, phi0_0_2pi_right,ref_phi0_right);


	//        // 腿部竖直力F的计算
	//        b_chassis.left_leg.leg_length_outer = pid_calc(&b_chassis.left_leg.leglengthpid_outer, b_chassis.left_leg.l0, ref_leglenth_left);
	//        b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid_inner, b_chassis.left_leg.dl0, b_chassis.left_leg.leg_length_outer);

	//        b_chassis.right_leg.leg_length_outer = pid_calc(&b_chassis.right_leg.leglengthpid_outer, b_chassis.right_leg.l0, ref_leglenth_right);
	//        b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid_inner, b_chassis.right_leg.dl0, b_chassis.right_leg.leg_length_outer);

	//        leg_conv(b_chassis.left_leg.leg_F, (Init_Tp_phi0_left ) / 2.0f, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
	//        leg_conv(b_chassis.right_leg.leg_F, (Init_Tp_phi0_right ) / 2.0f, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);


	//        motor_torque_set(DM4_POLARITY * b_chassis.right_leg.T[0],DM3_POLARITY * b_chassis.left_leg.T[0],DM2_POLARITY * b_chassis.left_leg.T[1],DM1_POLARITY * b_chassis.right_leg.T[1],0,0);

	//    }
	//    limit_outer();
	}

	/**
	************************************************************************************************************************
	* @Name     : balance_down_run_handle
	* @brief    : 倒地模式
	* @param	: down_flag
	* @retval   : void
	* @Note     : //腿部姿态到位 ->保持腿部姿态（保证轮子接触地面，摩擦力）->前进后退，转弯（最后加）
								要有双腿协调
	************************************************************************************************************************
	**/
	float Driving_t_l=0,Driving_t_r=0;
	u8 down_run_finish_flag;
	void balance_down_run_handle(void)
	{
			down_run_finish_flag=0;
			PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 400, 200, 68, 0, 0);      // 初始化初始化Tp的PID//500, 200, 40, 0, 60    200
			// 初始化左腿的长度PID参数  50 0 700
			PID_struct_init(&b_chassis.left_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 80, 0, 0);
			PID_struct_init(&b_chassis.left_leg.leglengthpid_outer, POSITION_PID, 10000, 20000, 30, 0, 0);    //气弹簧参数 85, 0, 10  25, 0, 0

			/***************************************************************************************************/
			// 初始化右腿的长度PID参数  50 0 700
			PID_struct_init(&b_chassis.right_leg.leglengthpid_inner, POSITION_PID, 20000, 20000, 80, 0, 0);
			PID_struct_init(&b_chassis.right_leg.leglengthpid_outer, POSITION_PID, 10000, 20000, 30, 0, 0);  //25 0 7

			VMC_data_get(&b_chassis.left_leg, balance_chassis.joint_Encoder[2].angle * DM3_POLARITY+2.18*PI/180.0f,
									 balance_chassis.joint_Encoder[2].gyro * DM3_POLARITY,
									 balance_chassis.joint_Encoder[1].angle * DM2_POLARITY + PI-2.18*PI/180.0f,
									 balance_chassis.joint_Encoder[1].gyro * DM2_POLARITY);

			VMC_data_get(&b_chassis.right_leg, balance_chassis.joint_Encoder[3].angle * DM4_POLARITY+2.18*PI/180.0f,
									 balance_chassis.joint_Encoder[3].gyro * DM4_POLARITY,
									 balance_chassis.joint_Encoder[0].angle * DM1_POLARITY + PI-2.18*PI/180.0f,
									 balance_chassis.joint_Encoder[0].gyro * DM1_POLARITY);

			float left_leg_phi1 = normalize_angle_pi(b_chassis.left_leg.phi1);
			float right_leg_phi1 = normalize_angle_pi(b_chassis.right_leg.phi1);
			float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0
			b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f) - chassis_gyro.pitch_Angle * PI / 180.0f);
			float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid, phi0, 0.87);
			// 双腿协调pid
			float harmonize_outer = pid_calc(&b_chassis.leg_harmonize_pid_outer, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
			float harmonize_inner = pid_calc(&b_chassis.leg_harmonize_pid_inner, (b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0), harmonize_outer);
			init_tp_calc(0.16f,harmonize_inner,Init_Tp);

			if (fabs(b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi) < PI / 2) // 如果云台角度与底盘偏航角差值小于PI/2,在同一象限内
			{
					target_angle = b_chassis.chassis_ref.remote_angle;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 3 * PI / 2) // 如果云台角度与底盘偏航角差值大于3*PI/2
			{
					target_angle = b_chassis.chassis_ref.remote_angle - 2 * PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll =  usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < -3 * PI / 2) // 如果差值的绝对值小于-3π/2
			{
					target_angle = b_chassis.chassis_ref.remote_angle + 2 * PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 0) // 如果差值的绝对值大于0
			{
					target_angle = b_chassis.chassis_ref.remote_angle - PI;
					ecd_speed = -b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll =  -usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < 0) // 如果差值的绝对值小于0
			{
					target_angle = b_chassis.chassis_ref.remote_angle + PI;
					ecd_speed = -b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
			}
			b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy, ecd_speed); // 将当前值调整到参考值
			b_chassis.chassis_ref.vw = - pid_calc(&b_chassis.pid_follow_gim, b_chassis.yaw_angle__pi_pi, target_angle);
			b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity; //

			vw_torque = pid_calc(&b_chassis.vw_pid, chassis_gyro.yaw_Gyro * PI / 180.0f, b_chassis.chassis_ref.vw);
			b_chassis.vw_limit_rate = 0.8;
			b_chassis.balance_loop.state_err[3] = b_chassis.chassis_ref.vy - b_chassis.balance_loop.dx;
			b_chassis.balance_loop.lqrOutT=b_chassis.balance_loop.state_err[3]*2;
			Driving_t_l = (b_chassis.balance_loop.lqrOutT / 2.0f + vw_torque * b_chassis.vw_limit_rate / 2.0f) * LEFT_WHEEL_POLARITY;
			Driving_t_r = (b_chassis.balance_loop.lqrOutT / 2.0f - vw_torque * b_chassis.vw_limit_rate / 2.0f) * RIGHT_WHEEL_POLARITY;

			motor_torque_set(DM4_POLARITY * b_chassis.right_leg.T[0],DM3_POLARITY * b_chassis.left_leg.T[0],DM2_POLARITY * b_chassis.left_leg.T[1],DM1_POLARITY * b_chassis.right_leg.T[1],Driving_t_l,Driving_t_r);

			// 电机输出限幅
			limit_outer();

	}

	/**
	************************************************************************************************************************
	* @Name     : chassis_stop_handle
	* @brief    : 停止模式，大幅用
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	void chassis_stop_handle(void)
	{
			b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;        // 将底盘的动态参考腿长赋值给底盘的参考腿长
			b_chassis.chassis_ref.vy = 0;                                                     // 设置底盘的参考速度为零 y轴方向速度
			b_chassis.chassis_ref.vx = 0;                                                     // x轴方向速度
			b_chassis.chassis_ref.vw = 0;                                                     // 设置底盘的参考角速度为零
			b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy * 0.0001 * TIME_STEP; // 设置底盘的参考位置为零, y轴位置增加量 = y轴方向速度 * 时间步长（0.001秒） * 时间步长常量
	}

	/**
	************************************************************************************************************************
	* @Name     : chassis_side_handle
	* @brief    : 侧向对敌
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	void chassis_side_handle(void)
	{
			PID_struct_init(&b_chassis.roll_pid_angle, POSITION_PID, 1, 1, 0.008, 0, 0.01); //// 初始化roll轴滚角的PID参数  0.012, 0.00002, 0.02  30  2.2, 0, 1
			PID_struct_init(&b_chassis.roll_leg_F_Rotate_pid, POSITION_PID, 20, 10, 2, 0, 1);

			PID_struct_init(&b_chassis.pid_follow_gim,POSITION_PID,6, 200, 8, 0, 1);// 初始化跟随云台的PID

			b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
			if (fabs(b_chassis.balance_loop.dx) > 0.5
							||b_chassis.chassis_ref.vy != 0
							||fabs(b_chassis.chassis_ref.vw) >= 1.7
							||usart_chassis_data.ctrl_mode==1)
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
			else
					b_chassis.normal_Y_erroffset-=b_chassis.balance_loop.dx*0.0003*TIME_STEP;

			if (fabs(b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi) < PI / 3) // 如果云台角度与底盘偏航角差值小于PI/2,在同一象限内
			{
					if(fabs(b_chassis.chassis_ref.remote_angle-PI/2)<PI/8)
					{
							target_angle = b_chassis.chassis_ref.remote_angle-PI/4;
							ecd_speed = b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = usart_chassis_data.roll;
					}
					else
					{
							if(fabs(b_chassis.chassis_ref.remote_angle+PI/2)<PI/8)
							{
									target_angle = b_chassis.chassis_ref.remote_angle+PI/4;
									ecd_speed = b_chassis.chassis_ref.remote_speed;
									b_chassis.chassis_ref.roll = usart_chassis_data.roll;
							}
							else
							{
									target_angle = b_chassis.chassis_ref.remote_angle;
									ecd_speed = b_chassis.chassis_ref.remote_speed;
									b_chassis.chassis_ref.roll = usart_chassis_data.roll;
							}
					}
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 3 * PI / 2) // 如果云台角度与底盘偏航角差值大于3*PI/2
			{
					target_angle = b_chassis.chassis_ref.remote_angle - 2 * PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll =  usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < -3 * PI / 2) // 如果差值的绝对值小于-3π/2
			{
					target_angle = b_chassis.chassis_ref.remote_angle + 2 * PI;
					ecd_speed = b_chassis.chassis_ref.remote_speed;
					b_chassis.chassis_ref.roll = usart_chassis_data.roll;
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi > 0) // 如果差值的绝对值大于0
			{
					if(fabs(b_chassis.chassis_ref.remote_angle-PI/2)<PI/8)
					{
							target_angle =  -b_chassis.chassis_ref.remote_angle +PI/4;
							ecd_speed =  -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll =  -usart_chassis_data.roll;
					}
					else
					{
							target_angle =  -b_chassis.chassis_ref.remote_angle;
							ecd_speed =  -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll =  -usart_chassis_data.roll;
					}
			}
			else if (b_chassis.chassis_ref.remote_angle - b_chassis.yaw_angle__pi_pi < 0) // 如果差值的绝对值小于0
			{
					if(fabs(b_chassis.chassis_ref.remote_angle + PI/2)<PI/8)
					{
							target_angle = b_chassis.chassis_ref.remote_angle + 3*PI/4;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					}
					else
					{
							target_angle = b_chassis.chassis_ref.remote_angle + PI;
							ecd_speed = -b_chassis.chassis_ref.remote_speed;
							b_chassis.chassis_ref.roll = -usart_chassis_data.roll;
					}
			}
			b_chassis.chassis_ref.vy = trackRamp(b_chassis.chassis_ref.vy, ecd_speed); // 将当前值调整到参考值
			b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_chassis_side,b_chassis.yaw_angle__pi_pi,target_angle);

			b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;

			if(b_chassis.rotate_flag==1)
			{
					if(fabs(chassis_gyro.yaw_Gyro * PI / 180.0f) >= 0.5)
					{
							b_chassis.chassis_ref.vy =0;
					}
					else
					{
							b_chassis.rotate_flag=0;
					}
			}
			VAL_LIMIT(b_chassis.chassis_ref.vy,-1.5,1.5);
	}
	/**
	************************************************************************************************************************
	* @Name     : chassis_seperate_handle
	* @brief    : 单个底盘模式，测试用
	* @param		: void
	* @retval   : void
	* @Note     :
	************************************************************************************************************************
	**/
	float sep_target_yaw;
	void chassis_seperate_handle(void)
	{
			if (seperate_cnt == 0) // 如果分离计数器为0
			{
					seperate_cnt++;                          // 计数
					sep_target_yaw = chassis_gyro.yaw_Angle; // 设置分离目标偏航角为底盘陀螺仪的偏航角
			}
			b_chassis.chassis_ref.vy = 0;   // 设置底盘参考速度的纵向分量为0
			b_chassis.chassis_ref.roll = 0; // 设置底盘参考姿态的滚动分量为0
			// 计算底盘旋转速度，使用PID控制器
			b_chassis.chassis_ref.vw = pid_calc(&b_chassis.pid_seperate_gim, chassis_gyro.yaw_Angle, sep_target_yaw);
			VAL_LIMIT(b_chassis.chassis_ref.vw, -5, 5); // 限幅 -5到5
			b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;

	}

	/**
	************************************************************************************************************************
	* @Name     : balance_task
	* @brief    : 平衡底盘解算
	* @param		: void
	* @retval   : void
	* @Note     :	一定要注意弧度制的转化啊
															电机极性要仔细检查
															检查好各个传感器的单位与性能
	************************************************************************************************************************
	**/
	void jump_task(void)
	{
//			b_chassis.chassis_ref.vx = 0; // 将在X轴方向上的线速度和绕Z轴的角速度初始化为0。
//			b_chassis.chassis_ref.vw = 0;
//			init_finish_flag=0;
			if (jump_state == 0)
			{

			//		PID_struct_init(&b_chassis.leg_harmonize_pid_inner, POSITION_PID, 45, 3, 11, 0, 1.0); //速度环
			//		PID_struct_init(&b_chassis.leg_harmonize_pid_outer, POSITION_PID, 50, 3, 35	, 0, 0.8); //角度环
			//		PID_struct_init(&b_chassis.pid_follow_gim,POSITION_PID,10, 200, 15, 0, 1);// 初始化跟随云台的PID


					jump_feedfront = 0;
					b_chassis.chassis_ref.leglength = 0.14f; // 设置腿长


					if (fabs(b_chassis.balance_loop.dx) > 0.5
									||b_chassis.chassis_ref.vy != 0
									||fabs(b_chassis.chassis_ref.vw) >= 3.0
									||usart_chassis_data.ctrl_mode==1)// 如果速度大于0.5或者底盘参考速度不为0或者串口控制模式为1或者底盘参考速度的绝对值大于0.2
							b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					} // 位置等于x，err为0
					else
					{

							b_chassis.normal_Y_erroffset -= b_chassis.balance_loop.dx * 0.0003 * TIME_STEP;

					}
			
					if(mea_distance<20&&jump_state==0&&mea_distance>0)
					{
						jump_state++;
						b_chassis.chassis_ref.leglength=0.38;
					}
						
				
	}
	/**
	************************************************************************************************************************
	* @Name     : balance_task
	* @brief    : 平衡底盘解算
	* @param		: void
	* @retval   : void
	* @Note     :	一定要注意弧度制的转化啊
															电机极性要仔细检查
															检查好各个传感器的单位与性能
	************************************************************************************************************************
	**/
	float rotato_offset=0,rotato_offset_kp=0;
	float current_fm=0,final_yaw_gyro=0;
	float ecd_dtheta = 0,final_dtheta = 0,current_fm_left,current_fm_right;
	// 最长到0.35，其实能到0.38
	float Tp_k=2.0f;//3.0f
	float right_leg_F_antifeedforward =18.0f;//28.0f;

 void balance_task_stand(void)
 {
	 VMC_data_get(&b_chassis.left_leg,
							     DaMiao_8009_Joint_2.ecd.angle  * DM2_POLARITY,
									 DaMiao_8009_Joint_2.ecd.gyro   * DM2_POLARITY, 
									 DaMiao_8009_Joint_3.ecd.angle  * DM3_POLARITY,
									 DaMiao_8009_Joint_3.ecd.gyro   * DM3_POLARITY
									 );
		
		
						VMC_data_get(&b_chassis.right_leg, 
									 DaMiao_8009_Joint_1.ecd.angle * DM1_POLARITY ,
									 DaMiao_8009_Joint_1.ecd.gyro  * DM1_POLARITY,
									 DaMiao_8009_Joint_4.ecd.angle * DM4_POLARITY,
									 DaMiao_8009_Joint_4.ecd.gyro  * DM4_POLARITY
									 );

			float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0

			ecd_dtheta = (((b_chassis.left_leg.dphi0 + b_chassis.right_leg.dphi0) / 2.0f) - chassis_gyro.pitch_Gyro * PI / 180.0f);
			if (isnan(ecd_dtheta) || isinf(ecd_dtheta))
			{
					ecd_dtheta = 0.0f; // 设置为一个合理的默认值
			}
			final_dtheta = Lpf_1st_calcu(&TEST_LPF, ecd_dtheta, 5, 0.002);
				b_chassis.balance_loop.phi = chassis_gyro.pitch_Angle * PI / 180.0f;

			b_chassis.balance_loop.dphi = chassis_gyro.pitch_Gyro * PI / 180.0f;
			b_chassis.balance_loop.x = ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R;
			b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity; //
			b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f) - chassis_gyro.pitch_Angle * PI / 180.0f);
			b_chassis.balance_loop.dtheta = ecd_dtheta;
			b_chassis.theta_left  = b_chassis.left_leg.phi0  - 1.57f - chassis_gyro.pitch_Angle * PI / 180.0f;
			b_chassis.theta_right = b_chassis.right_leg.phi0  - 1.57f - chassis_gyro.pitch_Angle * PI / 180.0f;
 }
 	float kp=0;
	float ki=0;
	float kd=0;
	void balance_task(void)
	{
						 		VMC_data_get(&b_chassis.left_leg,
							     DaMiao_8009_Joint_2.ecd.angle  * DM2_POLARITY,
									 DaMiao_8009_Joint_2.ecd.gyro   * DM2_POLARITY, 
									 DaMiao_8009_Joint_3.ecd.angle  * DM3_POLARITY,
									 DaMiao_8009_Joint_3.ecd.gyro   * DM3_POLARITY
									 );
		
		
						VMC_data_get(&b_chassis.right_leg, 
									 DaMiao_8009_Joint_1.ecd.angle * DM1_POLARITY ,
									 DaMiao_8009_Joint_1.ecd.gyro  * DM1_POLARITY,
									 DaMiao_8009_Joint_4.ecd.angle * DM4_POLARITY,
									 DaMiao_8009_Joint_4.ecd.gyro  * DM4_POLARITY
									 );

			float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f; // 计算Phi0

			ecd_dtheta = (((b_chassis.left_leg.dphi0 + b_chassis.right_leg.dphi0) / 2.0f) - chassis_gyro.pitch_Gyro * PI / 180.0f);
			if (isnan(ecd_dtheta) || isinf(ecd_dtheta))
			{
					ecd_dtheta = 0.0f; // 设置为一个合理的默认值
			}
			final_dtheta = Lpf_1st_calcu(&TEST_LPF, ecd_dtheta, 5, 0.002);

		if(b_chassis.ctrl_mode==CHASSIS_ROTATE)//
				b_chassis.balance_loop.phi = (chassis_gyro.pitch_Angle+0.5f) * PI / 180.0f;
		else
					b_chassis.balance_loop.phi = chassis_gyro.pitch_Angle * PI / 180.0f;

			b_chassis.balance_loop.dphi = chassis_gyro.pitch_Gyro * PI / 180.0f;
			b_chassis.balance_loop.x = ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].angle + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].angle) / 2.0f) * WHEEL_R;
			b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity; //
			b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0) / 2.0f) - 1.57f) - chassis_gyro.pitch_Angle * PI / 180.0f);
			b_chassis.balance_loop.dtheta = ecd_dtheta;
			b_chassis.theta_left  = b_chassis.left_leg.phi0  - 1.57f - chassis_gyro.pitch_Angle * PI / 180.0f;
			b_chassis.theta_right = b_chassis.right_leg.phi0  - 1.57f - chassis_gyro.pitch_Angle * PI / 180.0f;



			b_chassis.balance_loop.ddz = chassis_gyro.z_Acc * cos(chassis_gyro.pitch_Angle * PI / 180.0f);
			// 底盘轮子的平均线速度变化=平均角速度乘以半径
			b_chassis.balance_loop.wheel_dx = ((LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].gyro + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].gyro) / 2.0f) * WHEEL_R;
			// 底盘轮子的平均转速（RPM，即每分钟转数)
			b_chassis.balance_loop.RPM = (LEFT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[0].rate_rpm / REDUCTION_RATIO_16 + RIGHT_WHEEL_POLARITY * balance_chassis.Driving_Encoder[1].rate_rpm / REDUCTION_RATIO_16) / 2.0;
			// 腿长平均值
			b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0) / 2.0f;
			//向心力
			b_chassis.balance_loop.Fm = b_chassis.chassis_ref.vw * b_chassis.chassis_ref.vy * BODY_MASS;//（mv平衡转向时的向心力
			//实际向心力
			final_yaw_gyro = final_yaw_gyro* 0.98f + chassis_gyro.yaw_Gyro  * 0.02f;
			current_fm     = chassis_gyro.yaw_Gyro * PI / 180.0f*Mileage_kalman_filter.velocity*28.56;
//
		FN_Spring_left(&b_chassis.left_leg);
		FN_Spring_right(&b_chassis.right_leg);
			// 计算支持力
			FN_calculate(&b_chassis.left_leg,  DaMiao_8009_Joint_3.T_fdb*DM3_POLARITY,DaMiao_8009_Joint_2.T_fdb*DM2_POLARITY, &L_DDZW_LPF);  // capacitance_message.out_power
			FN_calculate(&b_chassis.right_leg,  DaMiao_8009_Joint_4.T_fdb*DM4_POLARITY,DaMiao_8009_Joint_1.T_fdb*DM1_POLARITY, &R_DDZW_LPF); // balance_chassis.Driving_Encoder[0].rate_rpm

			// lqr参数根据腿长的变化获取
			lqr_k(b_chassis.balance_loop.L0, b_chassis.balance_loop.K);
			for (int i = 0; i < 6; i++)
			{
					for (int j = 0; j < 2; j++)
					{

							b_chassis.balance_loop.k[j][i] = b_chassis.balance_loop.K[i * 2 + j];
					}
			}

			/********************************平衡部分解算********************************/
		if(midium_leg_flag==1)
		{
				b_chassis.balance_loop.state_err[0] = 0- b_chassis.balance_loop.theta;
		}
		else
					b_chassis.balance_loop.state_err[0] = THETA_OFFSET - b_chassis.balance_loop.theta;

	//		if(midium_leg_flag==1)
	//		{
	//				b_chassis.chassis_ref.pitch=PITCH_OFFSET-0.01;//抬头为正
	//		}
	//		else
					b_chassis.chassis_ref.pitch=PITCH_OFFSET;

			b_chassis.balance_loop.state_err[1] = 0 - b_chassis.balance_loop.dtheta;
			b_chassis.balance_loop.state_err[2] = b_chassis.chassis_ref.y_position - b_chassis.balance_loop.x;
			b_chassis.balance_loop.state_err[3] = b_chassis.chassis_ref.vy - b_chassis.balance_loop.dx;
			b_chassis.balance_loop.state_err[4] = b_chassis.chassis_ref.pitch - b_chassis.balance_loop.phi;


			b_chassis.balance_loop.state_err[5] = 0 - b_chassis.balance_loop.dphi;

			float x_err;
			if (jump_state >= 1)
			{
					x_err = 0;
			}
			else
					x_err = (b_chassis.balance_loop.state_err[2] + b_chassis.normal_Y_erroffset);
			// 对腿变化加速度的限制
	if (b_chassis.ctrl_mode == CHASSIS_ROTATE) // 如果是小陀螺模式
	{
			b_chassis.balance_loop.state_err[2] = 0; // 速度误差为0
	}
	else
			{   // 否则如果速度等于最大速度
					if (fabs(b_chassis.chassis_dynemic_ref.vy) == b_chassis.max_speed)
					{
							VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.3, 1.3); // 速度误差限幅为-1.5,1.5     //1.9
					}
					else
					{
							VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.8,1.8);   // 如果不相等限幅为-1.8,1.8  //1.5
					}  
			}
			// 平衡部分未离地力矩增益计算
			b_chassis.balance_loop.K_error[0][0] = b_chassis.balance_loop.k[0][0] * b_chassis.balance_loop.state_err[0];
			b_chassis.balance_loop.K_error[0][1] = b_chassis.balance_loop.k[0][1] * b_chassis.balance_loop.state_err[1];
			b_chassis.balance_loop.K_error[0][2] = b_chassis.balance_loop.k[0][2] * b_chassis.balance_loop.state_err[2];
			b_chassis.balance_loop.K_error[0][3] = b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
			b_chassis.balance_loop.K_error[0][4] = b_chassis.balance_loop.k[0][4] * b_chassis.balance_loop.state_err[4];
			b_chassis.balance_loop.K_error[0][5] = b_chassis.balance_loop.k[0][5] * b_chassis.balance_loop.state_err[5];

			b_chassis.balance_loop.K_error[1][0] = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0];
			b_chassis.balance_loop.K_error[1][1] = b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1];
			b_chassis.balance_loop.K_error[1][2] = b_chassis.balance_loop.k[1][2] * b_chassis.balance_loop.state_err[2];
			b_chassis.balance_loop.K_error[1][3] = b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3];
			b_chassis.balance_loop.K_error[1][4] = b_chassis.balance_loop.k[1][4] * b_chassis.balance_loop.state_err[4];
			b_chassis.balance_loop.K_error[1][5] = b_chassis.balance_loop.k[1][5] * b_chassis.balance_loop.state_err[5];
			float speed_k;
			if(fabs(left_acc)>310||fabs(right_acc)>310)//加速度的特殊情况处理
			{
					speed_k=0;
			}
			else
			{
					speed_k=b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
			}
			balance_Tgain = b_chassis.balance_loop.k[0][0] * b_chassis.balance_loop.state_err[0] +
											b_chassis.balance_loop.k[0][1] * b_chassis.balance_loop.state_err[1] +
											b_chassis.balance_loop.k[0][2] * x_err +
											speed_k                                                              +
											b_chassis.balance_loop.k[0][4] * b_chassis.balance_loop.state_err[4] +
											b_chassis.balance_loop.k[0][5] * b_chassis.balance_loop.state_err[5];

			balance_Tpgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] +
											 b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] +
											 b_chassis.balance_loop.k[1][2] * x_err +
											 b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3] +
											 b_chassis.balance_loop.k[1][4] * b_chassis.balance_loop.state_err[4] +
											 b_chassis.balance_loop.k[1][5] * b_chassis.balance_loop.state_err[5];

			// lqr离地增益计算
			V_T_outlandgain = 0;
			V_Tp_outlandgain = 0;
			balance_Toutlandgain = 0;
			balance_Tpoutlandgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1];

			if (isnan(b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0))
			{
					b_chassis.right_leg.dphi0 = 0.0f;
					b_chassis.left_leg.dphi0 = 0.0f;
			}
			// 双腿协调pid
			float harmonize_outer = pid_calc(&b_chassis.leg_harmonize_pid_outer, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
			float harmonize_inner = pid_calc(&b_chassis.leg_harmonize_pid_inner, (b_chassis.right_leg.dphi0 - b_chassis.left_leg.dphi0), harmonize_outer);

			// 转向pid
			vw_torque = pid_calc(&b_chassis.vw_pid, chassis_gyro.yaw_Gyro * PI / 180.0f, b_chassis.chassis_ref.vw);
			b_chassis.vw_limit_rate = 1;
		
			VAL_LIMIT(vw_torque, -3.5, 3.5);

			// roll平衡pid
			roll_F_output_angle = pid_calc(&b_chassis.roll_pid_angle, chassis_gyro.roll_Angle, b_chassis.chassis_ref.roll);
			roll_leg_F_output   = pid_calc(&b_chassis.roll_leg_F_Rotate_pid,chassis_gyro.roll_Angle, b_chassis.chassis_ref.roll-1.5);
//if ((wheel_state_estimate_left(&b_chassis.left_leg))&&(wheel_state_estimate_right(&b_chassis.right_leg)))
//{roll_leg_F_output=0;
//}
			b_chassis.left_leg.leg_length_outer  = pid_calc(&b_chassis.left_leg.leglengthpid_outer, b_chassis.left_leg.l0,(b_chassis.chassis_ref.leglength	)    );
			b_chassis.right_leg.leg_length_outer = pid_calc(&b_chassis.right_leg.leglengthpid_outer,b_chassis.right_leg.l0, (b_chassis.chassis_ref.leglength));

	
			// 腿部竖直力F的计算
			if(b_chassis.ctrl_mode==CHASSIS_ROTATE)
			{
					b_chassis.left_leg.leg_F  =  pid_calc(&b_chassis.left_leg.leglengthpid_inner, b_chassis.left_leg.dl0, b_chassis.left_leg.leg_length_outer)   + (BODY_MASS / 2) * 9.8 - roll_leg_F_output / 2 +b_chassis.balance_loop.Fm*0.5 ;
					b_chassis.right_leg.leg_F =  pid_calc(&b_chassis.right_leg.leglengthpid_inner, b_chassis.right_leg.dl0, b_chassis.right_leg.leg_length_outer) + (BODY_MASS / 2) * 9.8 + roll_leg_F_output / 2 -b_chassis.balance_loop.Fm*0.5  ;
			}
			else
			{
					if(jump_state == 1)
					{
							b_chassis.left_leg.leg_F  = jump_feedfront + b_chassis.left_leg.leg_length_outer + roll_leg_F_output / 2 +current_fm*0.5f ;
							b_chassis.right_leg.leg_F = jump_feedfront + b_chassis.right_leg.leg_length_outer- roll_leg_F_output / 2 - current_fm*0.5f ;
					}
					else
					{
							b_chassis.left_leg.leg_F  =  b_chassis.left_leg.leg_length_outer +b_chassis.left_leg.spring_FN + roll_leg_F_output / 2   + (BODY_MASS / 2) * 9.8 +current_fm*0.5f ;
							b_chassis.right_leg.leg_F =  b_chassis.right_leg.leg_length_outer+b_chassis.right_leg.spring_FN- roll_leg_F_output / 2  + (BODY_MASS / 2) * 9.8  - current_fm*0.5f ;
					}
			}
			b_chassis.balance_loop.lqrOutT = balance_Tgain;

			b_chassis.balance_loop.lqrOutTp = balance_Tpgain*2;

			// 此处的T0为phi1电机的扭矩，另一个是phi4的
			if (wheel_state_estimate_left(&b_chassis.left_leg)|| (b_chassis.ctrl_mode == CHASSIS_INIT)) // 如果左腿未离地或为初始化模式
			{
					// 设置左腿关节的扭矩，并计算左轮的驱动扭矩。
					leg_conv(b_chassis.left_leg.leg_F,(b_chassis.balance_loop.lqrOutTp - harmonize_inner) / 2.0f, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
					b_chassis.joint_T[1] = DM3_POLARITY * b_chassis.left_leg.T[0];
					b_chassis.joint_T[2] = DM2_POLARITY * b_chassis.left_leg.T[1];
					if(jump_state>=2)
					{
							b_chassis.driving_T[0]=0;
					}
				else
						b_chassis.driving_T[0] = (b_chassis.balance_loop.lqrOutT / 2.0f + vw_torque * b_chassis.vw_limit_rate / 2.0f) * LEFT_WHEEL_POLARITY;

			}
			else // 离地
			{
					b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					leg_conv(b_chassis.left_leg.leg_F,  (balance_Tpoutlandgain - harmonize_inner) / 2.0f, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
					b_chassis.joint_T[1] = DM3_POLARITY * b_chassis.left_leg.T[0];
					b_chassis.joint_T[2] = DM2_POLARITY * b_chassis.left_leg.T[1];
					b_chassis.driving_T[0] = 0;
			}
			// 右腿与左腿逻辑相同
			if (wheel_state_estimate_right(&b_chassis.right_leg) || (b_chassis.ctrl_mode == CHASSIS_INIT))
			{
					leg_conv(b_chassis.right_leg.leg_F, (b_chassis.balance_loop.lqrOutTp + harmonize_inner) / 2.0f, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
					b_chassis.joint_T[0] = DM4_POLARITY * b_chassis.right_leg.T[0];
					b_chassis.joint_T[3] = DM1_POLARITY * b_chassis.right_leg.T[1];
					if(jump_state>=2)
					{
							b_chassis.driving_T[1]=0;
					}
					else
							b_chassis.driving_T[1] = (b_chassis.balance_loop.lqrOutT / 2.0f - vw_torque * b_chassis.vw_limit_rate / 2.0f) * RIGHT_WHEEL_POLARITY;
			}
			else
			{
				b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
					leg_conv(b_chassis.right_leg.leg_F, (balance_Tpoutlandgain + harmonize_inner) / 2.0f, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
				b_chassis.joint_T[0] = DM4_POLARITY * b_chassis.right_leg.T[0];
				b_chassis.joint_T[3] = DM1_POLARITY * b_chassis.right_leg.T[1];
				b_chassis.driving_T[1] = 0;
			}


			VAL_LIMIT(b_chassis.joint_T[1], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.joint_T[2], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.joint_T[0], -JOINT_MAX_T, JOINT_MAX_T);
			VAL_LIMIT(b_chassis.joint_T[3], -JOINT_MAX_T, JOINT_MAX_T);
			if(if_middle_leg&&if_low_middle_leg==0)
			{
					VAL_LIMIT(b_chassis.driving_T[0], -4.0, 4.0);
					VAL_LIMIT(b_chassis.driving_T[1], -4.0, 4.0);
			}
			else
			{
					VAL_LIMIT(b_chassis.driving_T[0], -WHEEL_MAX_T, WHEEL_MAX_T);
					VAL_LIMIT(b_chassis.driving_T[1], -WHEEL_MAX_T, WHEEL_MAX_T);
			}


	}

	/**
	************************************************************************************************************************
	* @Name     : wheel_state_estimate
	* @brief    : 底盘离地检测函数
	* @param		: leg
	* @retval   : wheel_state
	* @Note     :
	************************************************************************************************************************
	**/
uint8_t wheel_state_estimate_left(leg_state_t *leg)
	{
			if (leg->leg_FN < 60) // 如果支持力小于20N 离地 轮子状态为0  leg->leg_FN < 20
			{

					leg->wheel_state = 0;
					return 0;
			}
			else // 未离地 轮子状态为1
			{
					leg->wheel_state = 1;
					return 1;
			}
	}
	uint8_t wheel_state_estimate_right(leg_state_t *leg)
	{
			if (leg->leg_FN < 60) // 如果支持力小于20N 离地 轮子状态为0  leg->leg_FN < 20
			{

					leg->wheel_state = 0;
					return 0;
			}
			else // 未离地 轮子状态为1
			{
					leg->wheel_state = 1;
					return 1;
			}
	}
					uint8_t wheel_state_estimate_sum(leg_state_t *leg_l,leg_state_t *leg_r)
	{
			if ((leg_l->leg_FN+leg_r->leg_FN) < 120) // 如果支持力小于20N 离地 轮子状态为0  leg->leg_FN < 20
			{

					leg_l->wheel_state = 0;
					leg_r->wheel_state = 0;
					return 0;
			}
			else // 未离地 轮子状态为1
			{
	leg_l->wheel_state =1;	
	leg_r->wheel_state =1;			
				
					return 1;
			}
	}

	float transform_angle_0_2pi(float angle)
	{
			float new_angle=fmod(angle+2*PI,2*PI);
			{
					return (new_angle<0)?new_angle+2*PI:new_angle;
			}
	}
	float normalize_angle_pi(float angle)
	{
			// 如果是无穷大或非数字，直接返回 0 或指定值
			if (isinf(angle) || isnan(angle))
			{
					return 0.0f; // 默认返回 0，可根据需要修改
			}

			// 使用 fmod 将角度归一化到 [-2PI, 2PI]
			angle = fmod(angle, 2 * PI);

			// 调整到 [-PI, PI] 范围
			if (angle > PI)
			{
					angle -= 2 * PI;
			}
			else if (angle < -PI)
			{
					angle += 2 * PI;
			}
			return angle;
	}
	void lqr_k(double L0, double K[12])
	{
			double t2;
			double t3;
			/* LQR_K_T
				K = LQR_K_T(L0)
				This function was generated by the Symbolic Math Toolbox version 23.2.    
	   Q=diag([ 100 200 300 400 400 40]);
    R=diag([7 17]);
			 2026-1-12 11:11 %
			*/

			t2 = L0 * L0;
			t3 = L0 * L0 * L0;

K[0] = ((L0 * -169.20782145770971 + t2 * 369.39433973475678) -
          t3 * 327.57177195060842) -
         6.5878751277186343;
  K[1] = ((L0 * 0.50094298538381632 - t2 * 70.239176737909915) +
          t3 * 99.862398608873178) +
         12.46821853098186;
  K[2] = ((L0 * -7.4793429548952233 - t2 * 1.5455442844058469) +
          t3 * 2.376941201892703) -
         0.52885114370810438;
  K[3] = ((L0 * 1.210355672732903 - t2 * 2.935465975427805) +
          t3 * 2.5614880499927879) +
         0.88325607014965757;
  K[4] = ((L0 * -17.716880949088608 + t2 * 44.488904538618762) -
          t3 * 41.128216470635827) -
         0.076161394870506258;
  K[5] = ((L0 * -11.526285590108159 + t2 * 19.891180638582881) -
          t3 * 14.04624143662315) +
         3.5121895657266982;
  K[6] = ((L0 * -21.190100035387911 + t2 * 50.374216960250457) -
          t3 * 45.733585211385368) -
         0.65511170403038366;
  K[7] = ((L0 * -17.8504858896031 + t2 * 34.763778184574853) -
          t3 * 27.71496766271224) +
         4.996298584806163;
  K[8] = ((L0 * -48.37126865216954 + t2 * 91.6204597395188) -
          t3 * 70.983111421410612) +
         13.585147880973571;
  K[9] = ((L0 * 72.71642588884022 - t2 * 197.36659839569069) +
          t3 * 191.79022228805309) +
         8.14339420254697;
  K[10] = ((L0 * -38.8413682324484 + t2 * 81.492931410012048) -
           t3 * 68.973097389090171) +
          9.99708298146584;
  K[11] = ((L0 * 54.368290576581551 - t2 * 150.00413297460011) +
           t3 * 147.54512609038881) +
          3.96199317395439;
			
}
void FN_Spring_left(leg_state_t *leg)
	{
			double t6,t5,t4,t3,t2,t1;
//		if(leg->l0>0.35)
//		t1=0.35;
//		else
			t1=leg->l0;
		t2=t1*t1;
		t3=t2*t1;
		t4=t3*t1;
		t5=t4*t1;
		t6=t5*t1;
		leg->spring_FN=
		1.6621e+07  * t6+
-2.2768e+07     * t5+
1.2606e+07      * t4+
-3.6012e+06     * t3+
5.5966e+05      * t2+
-4.5097e+04     * t1+
1.3704e+03;
		}
			void FN_Spring_right(leg_state_t *leg)
	{
			double t6,t5,t4,t3,t2,t1;
//	if(leg->l0>0.35)
//		t1=0.35;
//		else
			t1=leg->l0;
		t2=t1*t1;
		t3=t2*t1;
		t4=t3*t1;
		t5=t4*t1;
		t6=t5*t1;
		leg->spring_FN=
		4.8912e+06  * t6+
-6.3712e+06     * t5+
3.5030e+06      * t4+
-1.0436e+06     * t3+
1.7757e+05      * t2+
-1.6462e+04     * t1+
521.5205;
		} 

		
		
		

