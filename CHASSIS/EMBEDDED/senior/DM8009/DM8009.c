#include <main.h>


/****************** 结构体定义 ***********************/
DaMiao_8009_t DaMiao_8009_Joint_1;
DaMiao_8009_t DaMiao_8009_Joint_2;
DaMiao_8009_t DaMiao_8009_Joint_3;
DaMiao_8009_t DaMiao_8009_Joint_4;
/***************************************************/
/****************** 变量定义 ***********************/
uint16_t T_test=0;
float Torque_test=0;
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/********************************
*@Brief：   达妙电机 使能帧
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DaMiao_Motor_Enable(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
		
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFC;
	
	while((CANx->TSR&CAN_TSR_TME)==0);

	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg));
}

	
/********************************
*@Brief：   达妙电机 失能帧
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DaMiao_Motor_Disable(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
		
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFD;
	
CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg);
}


/********************************
*@Brief： DaMiao_8009 接收函数
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DaMiao_8009_Information_Receive(CanRxMsg *msg,int offset,DaMiao_8009_t *DaMiao_8009_Joint)
{
	DaMiao_8009_Joint->ecd.cal_data.can_cnt++;
	DaMiao_8009_Joint->ecd.cal_data.heart_cnt =time_tick;
		if(DaMiao_8009_Joint->ecd.cal_data.can_cnt<=5)
{DaMiao_8009_Joint->ecd.cal_data.ecd_bias = offset;}
	DaMiao_8009_Joint->ERR = NORMAL;
	int8_t ERR_Flag = (msg->Data[0]>>4);
	if(ERR_Flag == (int8_t)8)
	{
		DaMiao_8009_Joint->ERR = U_MAX;
	}
	if(ERR_Flag == (int8_t)9)
	{
		DaMiao_8009_Joint->ERR = U_MIN;
	}
	if(ERR_Flag == (int8_t)0x0A)
	{
		DaMiao_8009_Joint->ERR = I_MAX;
	}
	if(ERR_Flag == (int8_t)0x0B)
	{
		DaMiao_8009_Joint->ERR = MOS_TEM_MAX;
	}
	if(ERR_Flag == (int8_t)0x0C)
	{
		DaMiao_8009_Joint->ERR = ROTOR_TEM_MAX;
	}
	if(ERR_Flag == (int8_t)0x0D)
	{
		DaMiao_8009_Joint->ERR = CAN_BAG_LOST;
	}
	if(ERR_Flag == (int8_t)0x0E)
	{
		DaMiao_8009_Joint->ERR = OVERLOAD;
	}
	
	DaMiao_8009_Joint->P_fdb = (float)(((int16_t)((msg->Data[1]<<8) | (msg->Data[2]))) -32767);   //+-32767   
	DaMiao_8009_Joint->V_fdb = ((float)(((int16_t)((msg->Data[3]<<4) | (msg->Data[4]>>4)))-2047)) *0.02198 ;  //+-45r/s
	DaMiao_8009_Joint->T_fdb = ((float)(((int16_t)(((msg->Data[4]&0x0f)<<8) | (msg->Data[5])))-2047) )*0.01953125;
	DaMiao_8009_Joint->Temperature_MOS = msg->Data[6];
	DaMiao_8009_Joint->Temperature_Rotor = msg->Data[7];
	DaMiao_8009_Joint->rate_rpm=DaMiao_8009_Joint->V_fdb*30/PI;
	DaMiao_8009_Joint->Angle_fdb = DaMiao_8009_Joint->P_fdb *0.02186;//du 因为0.02186发现angle多了一倍，所以先除以二
	
	if(DaMiao_8009_Joint->Angle_fdb_last - DaMiao_8009_Joint->Angle_fdb > 720 ) //达妙系列圈数
	{
		DaMiao_8009_Joint->round_cnt++;
	}
	if(DaMiao_8009_Joint->Angle_fdb_last - DaMiao_8009_Joint->Angle_fdb < -720 ) 
	{
		DaMiao_8009_Joint->round_cnt--;
	}
//仅需计算gyro和angle,单位均为rad,可合并结构体至8009内，亦可保留Encoder结构体
DaMiao_8009_Joint->ecd.ecd_angle = (float)((DaMiao_8009_Joint->P_fdb - DaMiao_8009_Joint->ecd.cal_data.ecd_bias)*0.02197265625f  + DaMiao_8009_Joint->round_cnt * 1440);
DaMiao_8009_Joint->ecd.gyro = DaMiao_8009_Joint->V_fdb;
DaMiao_8009_Joint->ecd.angle = DaMiao_8009_Joint->ecd.ecd_angle*PI/180.0f;
	DaMiao_8009_Joint->Angle_fdb_last = DaMiao_8009_Joint->Angle_fdb;
	
	if (DaMiao_8009_Joint->ecd.cal_data.can_cnt <= 20)
	{
		if ((DaMiao_8009_Joint->ecd.cal_data.ecd_bias - DaMiao_8009_Joint->ecd.cal_data.ecd_value) < -32767)
		{
				DaMiao_8009_Joint->ecd.cal_data.ecd_bias = offset + 65536;
		}
		else if ((DaMiao_8009_Joint->ecd.cal_data.ecd_bias - DaMiao_8009_Joint->ecd.cal_data.ecd_value) > 32767)
		{
				DaMiao_8009_Joint->ecd.cal_data.ecd_bias = offset - 65536;
		}
	}
	
}


/********************************
*@Brief： DaMiao_8009 MIT模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DaMiao_8009_Information_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float Torque)
{	
	uint16_t Send_torque=0;
	CanTxMsg Motor_DaMiao_8009_CanTxMsg;

	Motor_DaMiao_8009_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_8009_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_8009_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_8009_CanTxMsg.DLC = 0x08;
		
	

//		DaMiao_8009_Joint->T_ref=0;

Send_torque=float_to_uint(Torque,-40,40,12);//实际使用
//Send_torque=float_to_uint(Torque_test,-40,40,12);//测试使用
	Motor_DaMiao_8009_CanTxMsg.Data[0] =0x7F; //DaMiao_8009_Joint->P_ref >>8;
	Motor_DaMiao_8009_CanTxMsg.Data[1] =0xFF; //DaMiao_8009_Joint->P_ref;
	Motor_DaMiao_8009_CanTxMsg.Data[2] =0x7F; //(int16_t)(DaMiao_8009_Joint->V_ref) >>4;
	Motor_DaMiao_8009_CanTxMsg.Data[3] =0xF0; //(((int16_t)(DaMiao_8009_Joint->V_ref)&0x000f)<<4) | ((DaMiao_8009_Joint->Kp&0x0f00)>>8);
	Motor_DaMiao_8009_CanTxMsg.Data[4] =0x00; //DaMiao_8009_Joint->Kp;
	Motor_DaMiao_8009_CanTxMsg.Data[5] =0x00; //(DaMiao_8009_Joint->Kd&0x0ff0)>>4;
	Motor_DaMiao_8009_CanTxMsg.Data[6] =(0x0F&(Send_torque>>8));//0x0000<<4|(((int16_t)(T_test)&0x0f00)>>8); //((DaMiao_8009_Joint->Kd&0x000f)<<4) | (((int16_t)(DaMiao_8009_Joint->T_ref)&0x0f00)>>8);
	Motor_DaMiao_8009_CanTxMsg.Data[7] =Send_torque; //DaMiao_8009_Joint->T_ref;
	
CAN_Transmit(CANx,&Motor_DaMiao_8009_CanTxMsg);
	
}


/********************************
*@Brief：   达妙电机 	保存位置零点
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DaMiao_8009_Position0_offset(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
	
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFE;
	
CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg);
}

