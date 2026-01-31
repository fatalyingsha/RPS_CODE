#include "can_chassis_transmit.h"


/**************************************************chassis_data************************************/
chassis_data_t can_chassis_data;

void can_chassis_send1(CAN_TypeDef *CANx)
{
	CanTxMsg tx_message;

	tx_message.StdId = UP_CAN2_TO_DOWN_CAN1_1;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;

	tx_message.Data[0] = can_chassis_data.if_follow_gim;
	tx_message.Data[1] = can_chassis_data.speed_mode;
	tx_message.Data[2] = can_chassis_data.chassis_mode;
	tx_message.Data[3] = (uint8_t)(can_chassis_data.rotate_speed >> 8);
	tx_message.Data[4] = (uint8_t)(can_chassis_data.rotate_speed);
	tx_message.Data[5] = (uint8_t)(can_chassis_data.yaw_Encoder_filter_rate >> 8);
	tx_message.Data[6] = (uint8_t)can_chassis_data.yaw_Encoder_filter_rate;
	tx_message.Data[7] = (uint8_t)can_chassis_data.chassis_power_limit;

	CAN_Transmit(CANx,&tx_message);
}

void can_chassis_send2(CAN_TypeDef *CANx)
{
	CanTxMsg tx_message;

	tx_message.StdId = UP_CAN2_TO_DOWN_CAN1_2;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;

	tx_message.Data[0] = (uint8_t)(can_chassis_data.x >> 8);
	tx_message.Data[1] = (uint8_t)can_chassis_data.x;
	tx_message.Data[2] = (uint8_t)(can_chassis_data.y >> 8);
	tx_message.Data[3] = (uint8_t)can_chassis_data.y;
	tx_message.Data[4] = (uint8_t)(can_chassis_data.chassis_power >> 8);
	tx_message.Data[5] = (uint8_t)can_chassis_data.chassis_power;
	tx_message.Data[6] = (uint8_t)(can_chassis_data.chassis_power_buffer >> 8);
	tx_message.Data[7] = (uint8_t)can_chassis_data.chassis_power_buffer;

	CAN_Transmit(CANx,&tx_message);
}

void can_chassis_send3(CAN_TypeDef *CANx)
{
	CanTxMsg tx_message;

	tx_message.StdId = UP_CAN2_TO_DOWN_CAN1_3;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	tx_message.Data[0] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 56);
	tx_message.Data[1] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 48);
	tx_message.Data[2] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 40);
	tx_message.Data[3] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 32);
	tx_message.Data[4] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 24);
	tx_message.Data[5] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 16);
	tx_message.Data[6] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 8);
	tx_message.Data[7] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle);
	
	CAN_Transmit(CANx,&tx_message);
}


void can_chassis_task(CAN_TypeDef *CANx,
										u8 if_follow_gim,
										u8 speed_mode,
										u8 chassis_mode,
										double yaw_encoder_angle,
										int16_t yaw_encoder_filter_rate,
										int16_t x,
										int16_t y,
										int16_t rotate_speed,
										int16_t chassis_power,
										uint16_t chassis_power_buffer,
										u8 chassis_power_limit)
{
	can_chassis_data.if_follow_gim = if_follow_gim;
	can_chassis_data.speed_mode = speed_mode;
	can_chassis_data.chassis_mode = chassis_mode;
	can_chassis_data.yaw_Encoder_ecd_angle = (int64_t)(yaw_encoder_angle*10000);
	can_chassis_data.yaw_Encoder_filter_rate = yaw_encoder_filter_rate;
	can_chassis_data.x = x;
	can_chassis_data.y = y;
	can_chassis_data.rotate_speed = rotate_speed;
	can_chassis_data.chassis_power = chassis_power;
	can_chassis_data.chassis_power_buffer = chassis_power_buffer;
	can_chassis_data.chassis_power_limit = chassis_power_limit;

	can_chassis_send1(CANx);
	can_chassis_send2(CANx);
	can_chassis_send3(CANx);
}


void can_chassis_receive_task(CanRxMsg * msg)
{
		switch (msg->StdId)
   {
			case UP_CAN2_TO_DOWN_CAN1_1:
		{
			
		}break;
			case UP_CAN2_TO_DOWN_CAN1_2:
		{
			
		}break;
		
			case UP_CAN2_TO_DOWN_CAN1_3:
		{
			
		}break;
	}
}

