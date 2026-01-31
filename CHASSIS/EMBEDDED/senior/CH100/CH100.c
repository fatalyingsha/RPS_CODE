#include "CH100.h"

/**
  ******************************************************************************
  * @file    CH100.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���д����CH100�����ǵ����ݽ�������㣬
                            ������ڲ���Ϊ����dma���յ�ַ��ͨ��������
                            �ṹ��

@verbatim
 ===============================================================================
 **/
void CH100_getDATA(uint8_t *DataAddress, general_gyro_t *GYRO)
{

    static __align(4) id0x91_t dat; /* struct must be 4 byte aligned */
    memcpy(&dat, &DataAddress[6], sizeof(id0x91_t));

    volatile static float Last_yaw_temp1, Yaw_temp1;
    volatile static int Yaw_count1;

    GYRO->pitch_Angle = dat.eul[1]-4.0f ;//1.5  4.0

    Last_yaw_temp1 = Yaw_temp1;
    Yaw_temp1 = dat.eul[2];
    if (Yaw_temp1 - Last_yaw_temp1 >= 324)
    {
        Yaw_count1--;
    }
    else if (Yaw_temp1 - Last_yaw_temp1 <= -324)
    {
        Yaw_count1++;
    }
    GYRO->yaw_Angle = -(Yaw_temp1 + Yaw_count1 * 360);
    // 将yaw_Angle从角度转为弧度，并限制在0到2π
    float yaw_angle_rad = GYRO->yaw_Angle * PI / 180.0; // 转为弧度

    // 将弧度值归一化到[0, 2PI]
    yaw_angle_rad = fmod(yaw_angle_rad, 2 * PI); // 取模操作
    if (yaw_angle_rad < 0)
    {
        yaw_angle_rad += 2 * PI; // 如果为负数，加2π
    }
    
    // 将归一化后的弧度值赋值回
    GYRO->yaw_Angle = yaw_angle_rad;
	
    GYRO->roll_Angle = dat.eul[0]-1.15f;

    GYRO->pitch_Gyro = dat.gyr[0]; // 0
   // float yaw_Gyro = dat.gyr[2];
	//GYRO->yaw_Gyro = GYRO->yaw_Gyro * 0.95f + yaw_Gyro * 0.05f;
	GYRO->yaw_Gyro = -dat.gyr[2];
    GYRO->roll_Gyro = dat.gyr[1]; // 1

    
    GYRO->x_Acc = (dat.acc[2] - cosf(dat.eul[0] * PI / 180.0) * cosf(dat.eul[1] * PI / 180.0)) * 9.81f;
    GYRO->y_Acc = (dat.acc[1] - sinf(dat.eul[1] * PI / 180.0)) * 9.81f;
    GYRO->z_Acc = (dat.acc[0] + sinf(dat.eul[0] * PI / 180.0) * cosf(dat.eul[1] * PI / 180.0)) * 9.81f;
	
	
}
