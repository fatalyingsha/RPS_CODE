#ifndef __GM_DOUBLELOOP_H
#define __GM_DOUBLELOOP_H
#include "public.h"



typedef struct
{  
 pid_t  test_pid_init_Angle;
pid_t  test_pid_init_speed;

float test_angle_ref;
float test_speed_ref;
float test_angle_fdb;
float test_speed_fdb;
  
 int16_t  gm_motor_input;
 int16_t  m3508_motor_input;
}motor_ref_and_fdb_t;

void pid_motor_task(void);
void pid_motor_init(void);
extern motor_ref_and_fdb_t  gmdata;
extern motor_ref_and_fdb_t  m3508data;
#endif 

