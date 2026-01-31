#include "remote_task.h"

u8 this_input_mode = 0;
u8 last_input_mode = 0;
Remote_chassis_t remote_chassis_data={0};
void remote_task(void)
{
	//切换遥控模式的时候所有任务归位重新开始
		last_input_mode = this_input_mode;
		this_input_mode = RC_CtrlData.inputmode;
		 if(RC_CtrlData.rc.ch4>1200&&b_chassis.overstep_flag==1)
	    {
			remote_chassis_data.leglength =0.36f ;
		}
		else
		{   
			remote_chassis_data.leglength =0.25f ;
		}
	    
	    VAL_LIMIT(remote_chassis_data.leglength,0.15,0.40);
	    
		if(this_input_mode != last_input_mode)
		{
			
            b_chassis.ctrl_mode = CHASSIS_RELAX;
			b_chassis.last_ctrl_mode = CHASSIS_RELAX;
			remote_chassis_data.ctr_mode = CHASSIS_RELAX;
			
		}
	
	switch((uint8_t)RC_CtrlData.inputmode)
	{
		case REMOTE_INPUT:
		{
			
			Remote_controlProcess();
		}break;
		case KEY_MOUSE_INPUT:
		{
			
		}break;
        case STOP:
        {
			 b_chassis.ctrl_mode = CHASSIS_RELAX;
			 remote_chassis_data.ctr_mode = CHASSIS_RELAX;
		}break;			
	}
}

void Remote_controlProcess(void)
{
	
	Remote_controlChassis();
}

void Remote_controlChassis(void)
{
	
	switch(RC_CtrlData.rc.s1)
	{
		case 1:
			 remote_chassis_data.ctr_mode = CHASSIS_ROTATE ;
		break;
		case 3:
			 remote_chassis_data.ctr_mode = CHASSIS_STOP;
		break;
		case 2:
			
			 remote_chassis_data.ctr_mode = MANUAL_FOLLOW_REMOTE;
		break;
	}
	
	if(RC_CtrlData.rc.ch4<800)
	{
		remote_chassis_data.overstep_cmd =1;
	}
	else
	{
		remote_chassis_data.overstep_cmd =0;
	}
//	if(RC_CtrlData.RemoteSwitch.s3to2)
//	{
//		remote_chassis_data.jump_cmd =1;;
//	}
//	else
//	{
//		remote_chassis_data.jump_cmd =0;
//	}
	if (remote_chassis_data.ctr_mode == CHASSIS_ROTATE)
	{
		
			remote_chassis_data.rotate_ref =  5;
	}
	
	
	
}
