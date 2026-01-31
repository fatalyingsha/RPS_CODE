#ifndef __USART_chassis_transmit_H
#define __USART_chassis_transmit_H
#include "public.h"


typedef struct
{
	u8 if_follow_gim;//是否跟随云台
	u8 jump_cmd;//跳跃命令
	u8 overstep_cmd;//上台阶命令
	u8 chassis_mode;//底盘模式
	float yaw_Encoder_ecd_angle;//yaw轴电机编码器角度
	float cmd_leg_length;//命令腿长
	float vx;
	float vy;
	float roll;
	int16_t rotate_speed;//小陀螺速度
	u8 ctrl_mode;//控制模式
	u8 remote_online_flag;
	u8 fric_wheel_run;
	u8 Rollover_posture_cmd;
	u8 low_speed_cmd;
	u8 UI_auto_aim_state;
	u8 gimbal_data_if_finish_Init;
	float leg_single_angle_handle_left;
	float leg_single_angle_handle_right;
	u8 fn_2_trigger_flag;
	u8 lock_shoot_check;

} usart_chassis_data_t;

typedef __packed struct
{
	uint16_t shooter_id1_17mm_cooling_heat;//
	uint16_t shooter_barrel_heat_limit;//射击热量上限
	uint16_t shooter_barrel_cooling_value;  //机器人射击热量每秒冷却值
	float    bullet_speed_x_hat;
	float    bullet_speed;
	uint8_t  robot_level;
	uint8_t  power_management_chassis_output;
	uint16_t current_HP;
    uint8_t  robot_id;
	uint8_t  allow_gimbal_init;
	int16_t remain_heat;
	uint8_t game_state;
		//以下调试用新加观测
	float chassis_theta;
		float leg_length_L;
	float leg_length_R;
	
	
} usart_gimbal_data_t;


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
					   uint8_t  game_state);
					   
void usart_gimbal_receive(usart_gimbal_data_t *data,uint8_t *DataAddress);	
void gimbal_control_online_detective(void);					   
extern usart_chassis_data_t usart_chassis_data;
extern usart_gimbal_data_t usart_gimbal_data;	
extern u8 gimbal_control_state;	
extern u8 gimbal_control_state_longtime;
					   
#endif
