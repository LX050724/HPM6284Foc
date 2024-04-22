
#include "iir_filter.h"
#include "hpm_common.h"
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>


void IIRFilterInit(IIRFilter_t *filter, uint32_t Sections, const float (*Matlab_NUM)[3], const float (*Matlab_DEN)[3])
{
    filter->Matlab_DEN = (float(*)[3])Matlab_DEN;
    filter->Matlab_NUM = (float(*)[3])Matlab_NUM;
    filter->Sections = Sections;
    filter->w = (float(*)[2])malloc(sizeof(float) * 2 * filter->Sections);
    for (uint32_t i = 0; i < filter->Sections; i++)
        filter->w[i][0] = filter->w[i][1] = 0.0f;
}

void IIRFilterDelete(IIRFilter_t *filter)
{
    free(filter->w);
    filter->Matlab_DEN = NULL;
    filter->Matlab_NUM = NULL;
    filter->w = NULL;
}

/**
 * @brief IIR滤波器单节运算
 * @param a 系数a向量
 * @param b 系数b向量
 * @param w 历史状态向量
 * @param input 信号输入
 * @return 滤波信号输出
 */
static ATTR_ALWAYS_INLINE float IIRFilter1Section(float *a, float *b, float s, float *w, float input)
{
    float t = input * s - a[0] * w[0] - a[1] * w[1];
    float r = t + b[0] * w[0] + b[1] * w[1];
    w[1] = w[0];
    w[0] = t;
    return r;
}

float ATTR_RAMFUNC IIRFilter(IIRFilter_t *filter, float input)
{
    for (uint32_t i = 0; i < filter->Sections; i++)
    {
        input = IIRFilter1Section(filter->Matlab_DEN[i * 2 + 1] + 1, filter->Matlab_NUM[i * 2 + 1] + 1,
                                  filter->Matlab_NUM[i * 2][0], filter->w[i], input);
    }
    return input * filter->Matlab_NUM[filter->Sections * 2][0];
}