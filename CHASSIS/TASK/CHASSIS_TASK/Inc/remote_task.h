#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H
#include "public.h"
#include "balance_task.h"

typedef struct
{
	float vx;
	float vy;
	float vw;
	float roll;
	float leglength;
	float angle;
	float rotate_ref;
	
	uint8_t last_ctr_mode;
	uint8_t ctr_mode;
	
	u8 jump_cmd;
	u8 overstep_cmd;
}Remote_chassis_t;

void remote_task(void);

void Remote_controlChassis(void);

void Remote_controlProcess(void);

extern Remote_chassis_t remote_chassis_data;






#endif



