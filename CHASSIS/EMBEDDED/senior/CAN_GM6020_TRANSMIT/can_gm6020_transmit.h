#ifndef __CAN_GM6020_TRANSMIT_H
#define __CAN_GM6020_TRANSMIT_H

#include "public.h"


typedef struct
{
    uint16_t angle_last;
    uint16_t angle;
    float angle_processed;
    int16_t speed;		// rpm
    int16_t current;
    int8_t  temp;
    
    int32_t number_turns;   // È¦Êý
}GM6020_t;



//gm6020
void can_gm6020_receive_task(CanRxMsg * msg);
void GM6020_SendByCAN(CAN_TypeDef *CANx,
                      int16_t data1, int16_t data2, int16_t data3, int16_t data4);
extern GM6020_t can_gm6020_data;

#endif

