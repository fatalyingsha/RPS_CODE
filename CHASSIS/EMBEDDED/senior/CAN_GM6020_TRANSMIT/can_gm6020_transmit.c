#include "can_gm6020_transmit.h"

GM6020_t can_gm6020_data;

void GM6020_SendByCAN(CAN_TypeDef *CANx,
                      int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
  CanTxMsg tx_message;
  tx_message.StdId = GM6020_SEND_ID1;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
  tx_message.Data[0] = (uint8_t)(data1 >> 8);
	tx_message.Data[1] = (uint8_t)data1;
	tx_message.Data[2] = (uint8_t)(data2 >> 8);
	tx_message.Data[3] = (uint8_t)data2;
	tx_message.Data[4] = (uint8_t)(data3 >> 8);
	tx_message.Data[5] = (uint8_t)data3;
	tx_message.Data[6] = (uint8_t)(data4 >> 8);
	tx_message.Data[7] = (uint8_t)data4;
	
	CAN_Transmit(CANx,&tx_message);
}

void can_gm6020_receive_task(CanRxMsg * msg)
{
  switch (msg->StdId)
  {
    case GM6020_RECEIVE_ID1:
		{
			can_gm6020_data.angle = ((msg->Data[0]<<8)|msg->Data[1]);
			can_gm6020_data.speed = ((msg->Data[2]<<8)|msg->Data[3]);
		  can_gm6020_data.current = ((msg->Data[4]<<8)|msg->Data[5]);
      can_gm6020_data.temp= msg->Data[6];
		}break;
        case GM6020_RECEIVE_ID3:
		{
			can_gm6020_data.angle = ((msg->Data[0]<<8)|msg->Data[1]);
			can_gm6020_data.speed = ((msg->Data[2]<<8)|msg->Data[3]);
		  can_gm6020_data.current = ((msg->Data[4]<<8)|msg->Data[5]);
      can_gm6020_data.temp= msg->Data[6];
		}break;
         case GM6020_RECEIVE_ID4:
		{
			can_gm6020_data.angle = ((msg->Data[0]<<8)|msg->Data[1]);
			can_gm6020_data.speed = ((msg->Data[2]<<8)|msg->Data[3]);
		  can_gm6020_data.current = ((msg->Data[4]<<8)|msg->Data[5]);
      can_gm6020_data.temp= msg->Data[6];
		}break;
         case GM6020_RECEIVE_ID2:
		{
			can_gm6020_data.angle = ((msg->Data[0]<<8)|msg->Data[1]);
			can_gm6020_data.speed = ((msg->Data[2]<<8)|msg->Data[3]);
		  can_gm6020_data.current = ((msg->Data[4]<<8)|msg->Data[5]);
      can_gm6020_data.temp= msg->Data[6];
		}break;
  }
}