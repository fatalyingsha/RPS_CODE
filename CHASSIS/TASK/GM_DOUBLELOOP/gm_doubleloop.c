#include "gm_doubleloop.h"


motor_ref_and_fdb_t  gmdata;
motor_ref_and_fdb_t  m3508data;


void pid_motor_init(void)
{
    gmdata.test_angle_ref=0;

    PID_struct_init(
        &gmdata.test_pid_init_Angle,
        POSITION_PID,
        350,
        1000,

        40,
        0,
        0);//18,0.01,0.5
    PID_struct_init(
        &gmdata.test_pid_init_speed,
        POSITION_PID,
        25000,
        15000,

        40,//70, 0.022,10
        4,
        2);
    PID_struct_init(
        &m3508data.test_pid_init_speed,
        POSITION_PID,
        8000,
        500,

        25,//25,0.022,1
        0.022,
        1);
    PID_struct_init(
        &m3508data.test_pid_init_Angle,
        POSITION_PID,
        350,
        1000,

        1,
        0,
        0);//18,0.01,0.5


}


void pid_motor_task(void)
{
    gmdata.test_speed_fdb = gm6020_Encoder.filter_rate;
    gmdata.test_angle_fdb = gm6020_Encoder.ecd_angle;
    gmdata.gm_motor_input=pid_double_loop_cal(&gmdata.test_pid_init_Angle,
                          &gmdata.test_pid_init_speed,
                          gmdata.test_angle_ref,
                          gmdata.test_angle_fdb,
                          &gmdata.test_speed_ref,
                          gmdata.test_speed_fdb,
                          0);

//GM6020_SendByCAN(CAN1,5000,5000,5000,5000);



    //m3508
    m3508data.test_speed_fdb = m3508_Encoder.filter_rate;
    m3508data.test_angle_fdb = m3508_Encoder.ecd_angle;
// m3508data.m3508_motor_input=pid_double_loop_cal(&m3508data.test_pid_init_Angle,
    //                          &m3508data.test_pid_init_speed,
//                          m3508data.test_angle_ref,
//                          m3508data.test_angle_fdb,
//													&m3508data.test_speed_ref,
//                          m3508data.test_speed_fdb,
//                          0);
    m3508data.m3508_motor_input=pid_calc(&m3508data.test_pid_init_speed,m3508data.test_speed_fdb,m3508data.test_speed_ref);

    Set_C620andC610_IQ1(CAN1,1500,3000,m3508data.m3508_motor_input,m3508data.m3508_motor_input);
}
