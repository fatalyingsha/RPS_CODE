#include "low_pass_filter.h"



Lpf1stObj ACC_LPF;
Lpf1stObj TEST_LPF={0};
Lpf1stObj L_DDZW_LPF,R_DDZW_LPF;
/**
 * @brief 计算一阶低通滤波器的输出值
 *
 * 使用一阶低通滤波算法计算输入信号u_k经过滤波后的输出值y_k。
 *
 * @param filter 指向一阶低通滤波器对象的指针
 * @param u_k 当前输入信号值
 * @param fc 滤波器的截止频率
 * @param ts 采样时间间隔
 * @return 经过低通滤波后的输出信号值y_k
 */


float Lpf_1st_calcu(Lpf1stObj *filter, float u_k,float fc, float ts)
{
    filter->alpha = 1 / (1 + 1/(2 * PI * fc * ts) );
    filter->ts = ts;
    filter->fc = fc;
    float y_k = filter->alpha*u_k + (1-filter->alpha)*filter->y_k1;
    filter->y_k1 = y_k;
    filter->u_k1 = u_k;
    return y_k;
}

