#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IIR滤波器结构体
 */
typedef struct
{
    float (*Matlab_DEN)[3];
    float (*Matlab_NUM)[3];
    float (*w)[2];
    uint32_t Sections;
} IIRFilter_t;

/**
 * @brief N阶IIR滤波器初始化
 * @param filter 滤波器
 * @param Sections 滤波器节数
 * @param Matlab_NUM Matlab导出头文件NUM系数
 * @param Matlab_DEN Matlab导出头文件DEN系数
 */
void IIRFilterInit(IIRFilter_t *filter, uint32_t Sections, const float (*Matlab_NUM)[3], const float (*Matlab_DEN)[3]);

/**
 * @brief N阶IIR滤波器
 * @param filter 滤波器
 * @param input 信号输入
 * @return 滤波信号输出
 */
float IIRFilter(IIRFilter_t *filter, float input);

/**
 * @brief 释放IIR滤波器内存
 * @param filter IIR滤波器
 */
void IIRFilterDelete(IIRFilter_t *filter);

#ifdef __cplusplus
}
#endif