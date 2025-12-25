#ifndef __FILTER_H__
#define __FILTER_H__

// 可以用RC低通滤波电路推导原理
// 推导出传递函数1/(1+RCS)，RC=时间常数，故1/RC为截止频率
// 再对传递函数进行z变换，即可推导出离散的一阶滤波公式
// 其中m_off_freq为截止频率，m_samp_tim为采样周期
class LowPassFilter
{
public:
    double m_preout;
    double m_out;
    double m_in;
    double m_off_freq; // 权重
    double m_samp_tim; // 采样步长

    double fir_a;

    LowPassFilter(){};
    ~LowPassFilter(){};
    LowPassFilter(double off_freq, double samp_tim) : m_off_freq(off_freq), m_samp_tim(samp_tim)
            { fir_a = 1 / (1 + m_off_freq * m_samp_tim); };
    void applyFilter(void);
    double applyFilter(double in);
};

#endif