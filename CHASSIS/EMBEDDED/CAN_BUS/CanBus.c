#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件用于配置can总线的发送与接收任务，若设置模块id可去头文件设置
						 
	* @notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
@verbatim
 ===============================================================================
 **/
 


void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
//		case JM1Encoder_MOTOR://右前
//		{

//			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[0],msg,JM1Encoder_Offset,0.017368678);//right 
//			
//		}
//        break;
//    
//			case JM2Encoder_MOTOR://左前
//		{

//			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[1],msg,JM2Encoder_Offset,0.017368678);//left phi4
//			
//		}
//        break;
//			case JM3Encoder_MOTOR://左后
//		{

//			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[2],msg,JM3Encoder_Offset,0.017368678);//left
//			
//		}
//        break;
//   
//    case JM4Encoder_MOTOR://右后
//		{

//			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[3],msg,JM4Encoder_Offset,0.017368678);//right
//			
//		}
//        break;
		case 0xA1:
			{
			DaMiao_8009_Information_Receive(msg,DM1Encoder_Offset,&DaMiao_8009_Joint_1);
			}
        break;
		case 0xA2:
			{
			DaMiao_8009_Information_Receive(msg,DM2Encoder_Offset,&DaMiao_8009_Joint_2);
			}
        break;
		case 0xA3:
			{
			DaMiao_8009_Information_Receive(msg,DM3Encoder_Offset,&DaMiao_8009_Joint_3);
			}
        break;
		case 0xA4:
			{
			DaMiao_8009_Information_Receive(msg,DM4Encoder_Offset,&DaMiao_8009_Joint_4);
			}
        break;
			
		default:
			break;
    }

}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
    case M3508_RECEIVE_ID1:
        /* code */
	{
		M3508orM2006EncoderTask(&balance_chassis.Driving_Encoder[0],msg);//左
	}
     break;
	case M3508_RECEIVE_ID2:
	{
		M3508orM2006EncoderTask(&balance_chassis.Driving_Encoder[1],msg);
	}
    break;

    default:
        break;
    }
//	Can_SuperCap_message_Process(&can_capacitance_message,msg);
}

float test_tor1=0;
float test_tor2=0;

float speed_ref_test=0;
void can_bus_send_task_1(void)
{
		 if(balance_chassis.Driving_Encoder[0].if_online&&balance_chassis.Driving_Encoder[1].if_online)//如果电机在线
    {
    if(b_chassis.ctrl_mode!=CHASSIS_RELAX)
		{
			
			DaMiao_8009_Information_Send(CAN1,0x01,b_chassis.joint_T[0]);//大小腿齿轮组交换，故同侧交叉赋值
			DaMiao_8009_Information_Send(CAN1,0x04,b_chassis.joint_T[3]);
			Set_C620andC610_IQ1(CAN2,b_chassis.driving_T[0]*TORQUE_CURRENT_CONSTANT_RATIO_16,b_chassis.driving_T[1]*TORQUE_CURRENT_CONSTANT_RATIO_16,	0,0); //16减速比
		}
		else
		{
			DaMiao_8009_Information_Send(CAN1,0x01,0.0);
		  DaMiao_8009_Information_Send(CAN1,0x04,0.0);
			Set_C620andC610_IQ1(CAN2,0,0,0,0);
		}
	}
		else
		{DaMiao_8009_Information_Send(CAN1,0x01,0.0);
		 DaMiao_8009_Information_Send(CAN1,0x04,0.0);
		}

}
void can_bus_send_task(void)
{
	  if(balance_chassis.Driving_Encoder[0].if_online&&balance_chassis.Driving_Encoder[1].if_online)//如果电机在线
    {
    if(b_chassis.ctrl_mode!=CHASSIS_RELAX)
		{
			DaMiao_8009_Information_Send(CAN1,0x02,b_chassis.joint_T[1]);//大小腿齿轮组交换，故同侧交叉赋值
			DaMiao_8009_Information_Send(CAN1,0x03,b_chassis.joint_T[2]);

		}
		else
		{
			DaMiao_8009_Information_Send(CAN1,0x02,0.0);
			DaMiao_8009_Information_Send(CAN1,0x03,0.0);

		}
	}
		else
		{DaMiao_8009_Information_Send(CAN1,0x02,0.0);
		 DaMiao_8009_Information_Send(CAN1,0x03,0.0);
		}

}  
	
void can_bus_read_right_offset(void)
{
			DaMiao_8009_Information_Send(CAN1,0x04,0.0);
			DaMiao_8009_Information_Send(CAN1,0x01,0.0);

			
}
void can_bus_read_left_offset(void)
{
	
	DaMiao_8009_Information_Send(CAN1,0x02,0.0);
  DaMiao_8009_Information_Send(CAN1,0x03,0.0);
}
