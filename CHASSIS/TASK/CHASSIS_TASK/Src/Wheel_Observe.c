#include "Wheel_Observe.h"

/**
  ******************************************************************************
  * @file    Mileage.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    28-November-2023
  * @brief   平衡步兵用里程计，该里程计使用卡尔曼滤波器将imu加速度计
							和轮式编码器进行融合，仅融合一个方向的速度和位置，用于
							平步的打滑检测。

@verbatim
 ===============================================================================
 **/

float prev_gyro_left,prev_gyro_right;

float difference_left_calc(float gyro,float dt)
{
    float acceleration = (gyro - prev_gyro_left) / dt;
	prev_gyro_left = gyro;
    return acceleration;

}

float difference_right_calc(float gyro,float dt)
{
    float acceleration = (gyro - prev_gyro_right) / dt;
	prev_gyro_right = gyro;
    return acceleration;

}