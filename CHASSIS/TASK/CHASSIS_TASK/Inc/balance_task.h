#ifndef  __BALANCE_TASK_H
#define  __BALANCE_TASK_H
#include "public.h"

//规定板子所放的位置为前
#define WHEEL_R 0.058
#define BODY_MASS 18.27//15.294//19.294
#define WHEEL_MASS 0.57//1.006 //1.3//556g*2
#define T_MAX 10 
#define WARNING_VOLTAGE 10
#define POWER_LIMIT 0
#define ROTATE_Y_ERROFFSET 0.0f
#define NORMAL_Y_ERROFFSET 0.8f //+0.40f
#define TIME_STEP 2
#define YAW_POLARITY 					-1 //逆正      舵轮要顺正，改-1；麦轮1
#define THETA_OFFSET 0.0f
#define PITCH_OFFSET -0.1f
#define LEFT_WHEEL_POLARITY             -1
#define RIGHT_WHEEL_POLARITY             1

#define OVERSTEP_THETA   0.3

//现在的
//#define JM1_POLARITY                    -1//右前电机极性
//#define JM2_POLARITY                    1//左前电机极性
//#define JM3_POLARITY                    1//左后电机极性
//#define JM4_POLARITY                    -1//右后电机极性
#define DM1_POLARITY										1
#define DM2_POLARITY                    -1
#define DM3_POLARITY                    -1
#define DM4_POLARITY                    1

#define JOINT_MAX_T                      40
#define WHEEL_MAX_T                      5.35 // 4.3

/**
************************************************************************************************************************
* @EnumName     : chassis_mode_e
* @brief        : This enumeration describes the various control modes of the chassis
* @param        : CHASSIS_RELAX     
* @param		: CHASSIS_STOP		
* @param		: MANUAL_SEPARATE_GIMBAL			
* @param		: MANUAL_FOLLOW_GIMBAL 		
* @param		: DODGE_MODE 		
* @param		: AUTO_SEPARATE_GIMBAL 	
* @param		: AUTO_FOLLOW_GIMBAL 	
* @param		: CHASSIS_ROTATE 		
* @param		: CHASSIS_REVERSE 	
* @param		: CHASSIS_CHANGE_REVERSE 		
* @param		: CHASSIS_SEPARATE 	
* @param		: CHASSIS_AUTO_SUP 		
* @Note     : 					
************************************************************************************************************************
**/
typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  CHASSIS_INIT 			 = 2,
  MANUAL_FOLLOW_REMOTE   = 3,
  CHASSIS_STAND_MODE     = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_DOWN_MODE      = 9,
  CHASSIS_SEPARATE 		 = 10,
  CHASSIS_AUTO_SUP       = 11,
  CHASSIS_SINGLE_LEG_HANDLE =12,
	CHASSIS_JUMP_TEST         =13,
	READYTORELAX              =14,
} chassis_mode_e;

typedef struct
{
	float theta;
	float dtheta;
	float ddtheta;
	float phi;
	float dphi;
	float x;
	float dx;
	float ddz;
	float wheel_dx;
	float Fm;
    float RPM;
	double K_error[2][6];
	double L0;
	double K[12];
	float k[2][6];
	float state_err[6];

	double lqrOutT;
	double lqrOutTp;
}lqr_system;

typedef struct
{
	float vx;
	float y_position;
	float vy;
	float vw;
	float remote_angle;
	float remote_speed;
	float roll;
	float pitch;
	float leglength;
}chassis_ref_t;

#ifndef LEG_STATE
#define LEG_STATE

typedef struct
{
	float pos[2];//pos=[l0; phi0];
	float spd[2];//spd[2]=[dl0; dphi0];
	float T[2];//T[2]=[motor4;motor1];

	//支持力解算用计算变量
	float J[4];
	float j[2][2];
	float F_fdb;
	float Tp_fdb;

	float phi4;
	float phi1;
	float dphi4;
	float dphi1;

	float this_dl0;
	float last_dl0;

	float l0;
	float dl0;
	float ddl0;
	float phi0;
	float dphi0;
	
    float leg_length_outer;
	float leg_F;
	float ddzw;
	float leg_FN;
	float temp_spring;
	float spring_FN;
	float leg_final_FN;
	
	uint8_t wheel_state;

	pid_t leglengthpid_inner;
	pid_t leglengthpid_outer;
}leg_state_t;


#endif // !LEG_STATE

typedef struct
{
	chassis_mode_e ctrl_mode;
	chassis_mode_e last_ctrl_mode;
	u8 jump_flag;
	u8 overstep_flag;
    u8 init_state;
	u8 rotate_flag;
	lqr_system balance_loop;
	chassis_ref_t chassis_ref;
	chassis_ref_t chassis_dynemic_ref;
    
	float vw_limit_rate;
	
	leg_state_t left_leg;
	leg_state_t right_leg;
	
	pid_t leg_harmonize_pid_inner;
	pid_t leg_harmonize_pid_outer;
	pid_t vw_pid;
	
	pid_t roll_pid_angle;
	pid_t roll_leg_F_pid;
	pid_t roll_leg_F_Rotate_pid;
	
	pid_t pid_follow_gim;
	
	
	pid_t pid_yaw_dist;
	pid_t pid_chassis_side;
	
	pid_t pid_seperate_gim;
	
	pid_t Init_Tp_pid;
	pid_t Init_Tp_dphi0_pid;
	pid_t Init_Tp_phi0_pid;
	
	pid_t over_step_phi0_left_pid;
	pid_t over_step_phi0_right_pid;
	
	pid_t Init_phi0_pid_left;
	pid_t Init_phi0_pid_right;
	
	
    pid_t Init_dphi0_pid_left;
	pid_t Init_dphi0_pid_right;
	u16 Max_power_to_PM01;
	
	float theta_left;
    float theta_right;

	
	double yaw_encoder_ecd_angle;
	float yaw_angle_0_2pi;
	float yaw_angle__pi_pi;
	float normal_Y_erroffset;
	float remote_ref_vx;
	
	float predict_power;
	float max_speed;
	float min_speed;
	//tqouce
	double joint_T[4];
	double driving_T[2];
}Balance_chassis_t;

void get_remote_angle(void);
void chassis_Init_handle(void);
void init_tp_calc(float ref_leglength,float harmonize,float init_Tp);
void motor_torque_set(float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2);

void chassis_follow_gyro(void);
void chassis_seperate_handle(void);
void balance_over_slope_handle(void);

void jump_task(void);
void balance_task(void);
void chassis_cmd_selest(void);
void lqr_k(double L0, double K[12]);
float normalize_angle_pi(float angle);
float transform_angle_0_2pi(float angle);
void chassis_side_handle(void);
void chassis_standup_handle(void);
void chassis_stop_handle(void);
void stand_up_test(void);
void chassis_test_all(void);
void chassis_task(void);
void chassis_follow_gim_handle(void);
void chassis_rotate_handle(void);
void balance_jump_handle(void);

void yaw_control_update(float mea_distance_left,float mea_distance_right);


void balance_jump_distance_yaw_handle(void);

void balance_jump_double_handle(void);


void balance_down_run_handle(void);
void chassis_single_leg_handle(void);
 
void balance_step_handle(void);
void balance_param_init(void);
void lqr_k(double L0, double K[12]);
void FN_Spring_left(leg_state_t *leg);
void FN_Spring_right(leg_state_t *leg);
uint8_t wheel_state_estimate_left(leg_state_t* leg);
uint8_t wheel_state_estimate_right(leg_state_t* leg);
uint8_t wheel_state_estimate_sum(leg_state_t *leg_l,leg_state_t *leg_r);

void limit_outer(void);
extern Balance_chassis_t b_chassis;
extern u8 jump_state;
extern u8 down_run_finish_flag;
extern uint8_t allow_gimbal_init;
extern u8 state_flag_2;
extern u8 state_flag_1;

extern u8 jump_state_double;
extern uint8_t step_state;
extern uint8_t step_flag;
extern uint16_t step_state_cnt;
extern uint8_t init_abnormal_state;
extern float jump_feedfront;
#endif // ! __BALANCE_TASK_H
