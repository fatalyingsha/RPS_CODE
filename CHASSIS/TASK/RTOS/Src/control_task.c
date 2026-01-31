#include "control_task.h"

int torque=0;
int time_tick = 0;

void control_task(void)
{
    time_tick++;
    //电机在线监测
	online_detective(&balance_chassis.Driving_Encoder[0]);
	online_detective(&balance_chassis.Driving_Encoder[1]);
 //   TF02_online_detective();
//	gimbal_control_online_detective();
	JUDGE_TASK();
	chassis_test_all();//底盘控制任务

    if(time_tick%2==0)
    {//inner实际给到init的Tp传递中
			
					
	//			can_bus_read_left_offset();
		can_bus_send_task();
		
    }

		if(time_tick%2==1)
		{	
	 	can_bus_send_task_1();
		//	can_bus_read_right_offset();
//			if(RC_CtrlData.rc.s2==1)
//				
//			{					DaMiao_8009_Information_Send(CAN1,0x04,-24);//左前
//			DaMiao_8009_Information_Send(CAN1,0x01,24);//左后
//			}
//			if(RC_CtrlData.rc.s2==3)
//			{
//			DaMiao_8009_Information_Send(CAN1,0x04,-3);//左前
//			DaMiao_8009_Information_Send(CAN1,0x01,3);//左后
//	//		DaMiao_8009_Information_Send(CAN1,0x02,-3);//左前
//	//	DaMiao_8009_Information_Send(CAN1,0x03,3);//左后
//			}
//			if(RC_CtrlData.rc.s2==2)
//				
//		{		
////				DaMiao_8009_Information_Send(CAN1,0x02,0);//左前
////			DaMiao_8009_Information_Send(CAN1,0x03,0);//左后
//			DaMiao_8009_Information_Send(CAN1,0x01,0);//左前
//		DaMiao_8009_Information_Send(CAN1,0x04,0);//左后
//		}
		
		}
    if(time_tick%5==0)
    {
      usart_gimbal_send(
		                judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat,
		                judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit,
		                judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value,
		                judge_rece_mesg.game_robot_state.robot_level,
		                Bullet_Speed_Kalman.X_hat,
		                judge_rece_mesg.shoot_data.bullet_speed,
		                judge_rece_mesg.game_robot_state.power_management_chassis_output,
		                judge_rece_mesg.game_robot_state.current_HP,
		                judge_rece_mesg.game_robot_state.robot_id,
		                allow_gimbal_init,
		                judge_rece_mesg.game_state.game_progress);//发至云台
	
    }
    if(time_tick % 10 ==9)
    {
		 
	     //  POWER_Control(&Super_Cap_Send);
 		   CAN_POWER_Control(CAN2,&Super_Cap_Send);
    }
	
	if (time_tick % 100 == 0) // 上传用户信息
	{
		Client_Send_Handle();
	}
	 if(time_tick%100 == 50) //上传用户信息
    {
      Send_bullet_remaining_num();
    }
	
	if(time_tick%1000==0)
		{
			HARD_WDG_ON;
		}
		if(time_tick%2000==0)
		{
			HARD_WDG_OFF;
		}
}
void control_task_Init(void)
{
    balance_param_init();
}
