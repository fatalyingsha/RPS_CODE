#include "USART_chassis_transmit.h"

usart_chassis_data_t usart_chassis_data;
usart_gimbal_data_t usart_gimbal_data;
u8 databuff[100];
u8 gimbal_control_state;
u8 gimbal_control_state_longtime;

int32_t gimbal_control_online_heart_cnt;


void usart_chassis_receive(uint8_t *DataAddress)
{
    usart_chassis_data.yaw_Encoder_ecd_angle = ((int32_t)(((DataAddress[11]<<24)|(DataAddress[12]<<16)|(DataAddress[13]<<8)|DataAddress[14])))/10000.0f;
	usart_chassis_data.if_follow_gim = DataAddress[0];
	usart_chassis_data.jump_cmd = DataAddress[1];
	usart_chassis_data.chassis_mode = DataAddress[2];
	usart_chassis_data.cmd_leg_length = ((int16_t)((DataAddress[3]<<8)|DataAddress[4]))/100.0;
	usart_chassis_data.vx = ((int16_t)((DataAddress[5]<<8)|DataAddress[6]))/100.0;
	usart_chassis_data.vy = ((int16_t)((DataAddress[7]<<8)|DataAddress[8]))/100.0;
	usart_chassis_data.rotate_speed = ((DataAddress[9]<<8)|DataAddress[10]);
	usart_chassis_data.ctrl_mode = DataAddress[15];
    usart_chassis_data.roll = ((int16_t)((DataAddress[16]<<8)|DataAddress[17]))/100.0;
	usart_chassis_data.overstep_cmd = DataAddress[18];
	usart_chassis_data.remote_online_flag=DataAddress[19];
	usart_chassis_data.fric_wheel_run  =DataAddress[20];
	usart_chassis_data.Rollover_posture_cmd =DataAddress[21];
	usart_chassis_data.low_speed_cmd =DataAddress[22];
	usart_chassis_data.UI_auto_aim_state =DataAddress[23];
	usart_chassis_data.gimbal_data_if_finish_Init=DataAddress[24];
    usart_chassis_data.leg_single_angle_handle_left = ((int16_t)((DataAddress[25]<<8)|DataAddress[26]))/100.0;
    usart_chassis_data.leg_single_angle_handle_right = ((int16_t)((DataAddress[27]<<8)|DataAddress[28]))/100.0;
	usart_chassis_data.fn_2_trigger_flag = DataAddress[29];
	usart_chassis_data.lock_shoot_check = DataAddress[30];
	
    gimbal_control_online_heart_cnt=time_tick;
	

	
}


void gimbal_control_online_detective(void)
{
    if(abs(time_tick - gimbal_control_online_heart_cnt) > 500)
    {
        gimbal_control_state = 0;
		
    }else
    {
        gimbal_control_state = 1;
    }
	
	 if(abs(time_tick - gimbal_control_online_heart_cnt) > 8000)
    {
        gimbal_control_state_longtime=0;		
    }else
    {
        gimbal_control_state_longtime=1;
    }
}

void usart_gimbal_send(
					   uint16_t shooter_id1_17mm_cooling_heat,
	                   uint16_t shooter_barrel_heat_limit,
	                   uint16_t shooter_barrel_cooling_value,
	                   uint8_t  robot_level,
                       float    bullet_speed_x_hat,
					   float    bullet_speed,
                       uint8_t  power_management_chassis_output,
					   uint16_t current_HP,
					   uint8_t  robot_id,
					   uint8_t  allow_gimbal_init,
					   uint8_t  game_state)
{
	usart_gimbal_data.shooter_id1_17mm_cooling_heat  = shooter_id1_17mm_cooling_heat;
	usart_gimbal_data.shooter_barrel_heat_limit      = shooter_barrel_heat_limit;
	usart_gimbal_data.shooter_barrel_cooling_value   = shooter_barrel_cooling_value;
	usart_gimbal_data.robot_level                    = robot_level;
	usart_gimbal_data.bullet_speed_x_hat             = bullet_speed_x_hat; 
	usart_gimbal_data.bullet_speed                   = bullet_speed;
	usart_gimbal_data.power_management_chassis_output= power_management_chassis_output;
	usart_gimbal_data.current_HP                     = current_HP;
	usart_gimbal_data.robot_id                       = robot_id;
	usart_gimbal_data.allow_gimbal_init              = allow_gimbal_init;
	usart_gimbal_data.remain_heat                    = remain_heat;
	usart_gimbal_data.game_state                     = game_state;
	usart_gimbal_data.chassis_theta                  = b_chassis.balance_loop.theta;     
	usart_gimbal_data.leg_length_L 									 = b_chassis.left_leg.l0;
	usart_gimbal_data.leg_length_R									 = b_chassis.right_leg.l0;
	remain_heat = 0;
	
    usart4.Send_bytes(&usart4,(uint8_t *)&usart_gimbal_data,sizeof(usart_gimbal_data_t));
 	
	
}

void usart_gimbal_receive(usart_gimbal_data_t *data,uint8_t *DataAddress)
{
	memcpy((uint8_t *)data,DataAddress,sizeof(usart_gimbal_data_t));
}
