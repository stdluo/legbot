#include "../../include/common/Filter.h"

// （1-fir_a）为滤波系数
// 滤波系数越小，滤波结果越平稳，但是灵敏度越低；
// 滤波系数越大，灵敏度越高，但是滤波结果越不稳定
void LowPassFilter::applyFilter(void)
{
    m_out = fir_a * m_preout + (1 - fir_a) * m_in;
    m_preout = m_out;
}

double LowPassFilter::applyFilter(double in)
{
    m_in = in;
    m_out = fir_a * m_preout + (1 - fir_a) * m_in;
    m_preout = m_out;
    return m_out;
}