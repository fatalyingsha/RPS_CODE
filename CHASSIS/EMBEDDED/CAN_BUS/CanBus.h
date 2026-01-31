#ifndef __CANBUS_H
#define __CANBUS_H
#include "public.h"
/*********************************MG8016*********************************/


#define JM1Encoder_MOTOR 0x141   //象限分布
#define JM2Encoder_MOTOR 0x142
#define JM3Encoder_MOTOR 0x143
#define JM4Encoder_MOTOR 0x144

/*********************************GM6020id*********************************/
#define GM6020_SEND_ID1 0x1FF
#define GM6020_SEND_ID2 0x2FF

#define GM6020_RECEIVE_ID(id)   (0x204+(id))

#define GM6020_RECEIVE_ID1     GM6020_RECEIVE_ID(1) 
#define GM6020_RECEIVE_ID2     GM6020_RECEIVE_ID(2) 
#define GM6020_RECEIVE_ID3     GM6020_RECEIVE_ID(3) 
#define GM6020_RECEIVE_ID4     GM6020_RECEIVE_ID(4) 
/*********************************M3508 id*********************************/
#define M3508_MASTER_ID(id)				(0x200 + (id))
/* CAN总线发送报文ID */
#define M3508_SEND_ID1      0x200
#define M3508_SEND_ID2      0x1FF

#define M3508_RECEIVE_ID1      M3508_MASTER_ID(1)
#define M3508_RECEIVE_ID2      M3508_MASTER_ID(2)
#define M3508_RECEIVE_ID3      M3508_MASTER_ID(3)
#define M3508_RECEIVE_ID4      M3508_MASTER_ID(4)
/**********************舵轮底盘电机id||麦轮底盘电机id**************************************/
#define GM1Encoder_MOTOR 0x00
#define GM2Encoder_MOTOR 0X00
#define GM3Encoder_MOTOR 0X00
#define GM4Encoder_MOTOR 0X00

#define CM1Encoder_MOTOR 0x00
#define CM2Encoder_MOTOR 0x00
#define CM3Encoder_MOTOR 0x00
#define CM4Encoder_MOTOR 0x00
/*************************云台电机id******************************/
#define GIMBAL_YAW_MOTOR 0x00
#define GIMBAL_PITCH_MOTOR 0X00
/****************************英雄小云台电机id***********************************/
#define SMALL_GIMBAL_MOTOR 0X00
#define SCOPE_MOTOR 0X00
/*********************************摩擦轮电机id**************************************/
#define LEFT_FRICTION 0X202
#define RIGHT_FRICTION 0X201
//哨兵
#define LEFT_UP_FRICTION 0X00
#define RIGHT_UP_FRICTION 0X00
#define LEFT_DOWN_FRIICTION 0X00
#define RIGHT_DOWN_FRICTION 0X00
/**********************************拨盘电机id**************************************/
#define DOWN_POKE 0x205    
#define UP_POKE 0X205
#define LEFT_POKE 0X00			//右一左二，只有一个用一
#define RIGHT_POKE 0X00
#define POKE 0X00
/*********************************舵轮上下板通信id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0X409



void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);



void can_bus_read_right_offset(void);		
void can_bus_read_left_offset(void);		
void can_bus_send_task(void);

#endif
