#ifndef __DM8009_H_
#define __DM8009_H_

#include "stm32f4xx.h" 
/*dps 度每秒*/
/*filter_rate rpm 转每分*/

/************************************************** 结构体定义 *****************************************************************/
//首字母大写，_e枚举 _t结构体
typedef enum
{
	NORMAL			=0,
	U_MAX       =1,
	U_MIN				=2,
	I_MAX				=3,
	MOS_TEM_MAX		=4,   	//MOS 过温
	ROTOR_TEM_MAX	=5,			//电机线圈过温
	CAN_BAG_LOST	=6,			//通讯丢失
	OVERLOAD			=7,			//过载
}DaMiao_ERR_e;								//故障类型
 
 
typedef struct
{
	int16_t  ID;
	DaMiao_ERR_e ERR;
	
	int16_t Kp;					// Kp 的范围为[0,500]，Kd 的范围为[0,5]
	int16_t Kd;					// P*Kp + V*Kd = T_ref—> I_ref
	
	int16_t P_fdb;				//当前位置  rad
	int16_t P_ref;				//当前位置
	
	float P_des; 			//发送给电机的位置值
	float V_des; 			//发送给电机的最大速度值
	
	
	float Angle_fdb;			//当前角度   °
	float Angle_ref;			//目标角度
	float Angle_fdb_last;   //电机上一次角度 
	
	
	float V_fdb;				//电机当前转速  rad/s
	float V_ref;				//电机目标转速
	
	float T_fdb;					//电机的扭矩信息   N·M	
	float T_ref;					//电机的扭矩信息
	
	int16_t rate_rpm; //rpm 转每分
	
	uint16_t Temperature_MOS;		//驱动上 MOS 的平均温度，单位℃
	uint16_t Temperature_Rotor;		//电机内部线圈的平均温度，单位℃
	
	int16_t round_cnt;  //圈数
	int8_t  Flag_init_direction;		//初始化时方向是正是负，解算±180在0°上
	double angle;
	double gyro;
	Encoder ecd;
}DaMiao_8009_t;

/******************************************************************************************************************************/


/******************* 结构体or变量的声明 **********/

extern DaMiao_8009_t DaMiao_8009_Joint_1;
extern DaMiao_8009_t DaMiao_8009_Joint_2;
extern DaMiao_8009_t DaMiao_8009_Joint_3;
extern DaMiao_8009_t DaMiao_8009_Joint_4;



/************************************************/


/****************** 函 数 声 明 ********************************/
void DaMiao_Motor_Enable(CAN_TypeDef* CANx, int16_t CAN_ID);		//使能
void DaMiao_Motor_Disable(CAN_TypeDef* CANx, int16_t CAN_ID);		//失能


void DaMiao_8009_Information_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float Torque);
void DaMiao_8009_Position_Send(CAN_TypeDef* CANx, DaMiao_8009_t *DaMiao_8009_Joint_1,int16_t CAN_ID);
void DaMiao_8009_Position0_offset(CAN_TypeDef* CANx, int16_t CAN_ID);

void DaMiao_8009_Information_Receive(CanRxMsg *msg,int offset,DaMiao_8009_t *DaMiao_8009_Joint);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
/***************************************************************/


#endif  /* _DAMIAO_8009_H_ */
