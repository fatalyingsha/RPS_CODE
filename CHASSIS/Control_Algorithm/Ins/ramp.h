#ifndef __ramp_H__
#define __ramp_H__
#include "public.h"

#define MAX_DELTA 0.02
#define MAX_DELTA_LEG 0.001



float trackRamp(float current, float reference);

float trackRamp_leg(float max_delta_leg ,float current, float reference); 
#endif

