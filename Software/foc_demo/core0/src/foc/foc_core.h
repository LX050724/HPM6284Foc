#pragma once

#include <stdint.h>
#include "fast_sin.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float iu;
    float iv;
    float iw;
} foc_uvw_current_t;

typedef struct
{
    float i_alpha;
    float i_beat;
} foc_alpha_beta_current_t;

typedef struct
{
    float iq;
    float id;
} foc_qd_current_t;

typedef struct
{
    float v_alpha;
    float v_beta;
} foc_alpha_beta_volt_t;

typedef struct
{
    uint32_t pwm_u;
    uint32_t pwm_v;
    uint32_t pwm_w;
} foc_pwm_t;

typedef struct
{
    float sinx;
    float cosx;
} foc_sin_cos_t;

/**
 * @brief 快速三角函数
 * 
 * @param x 输入弧度
 * @param ang 输出正弦余弦值
 */
static inline void foc_sin_cos(float x, foc_sin_cos_t *ang)
{
    fast_sin_cos(x, &ang->sinx, &ang->cosx);
}

/**
 * @brief 0矢量注入法SVPWM
 * 
 * @param volt [in] 输入电压，有效范围[-1,1]
 * @param pwm [out] 输出PWM值
 */
void foc_svpwm(const foc_alpha_beta_volt_t *volt, foc_pwm_t *pwm);

/**
 * @brief park变换
 * 
 * @param in alpha beta电流
 * @param ang 电角度正余弦
 * @param out qd电流
 */
void foc_park(const foc_alpha_beta_current_t *in, const foc_sin_cos_t *ang, foc_qd_current_t *out);

/**
 * @brief park逆变换
 * 
 * @param in qd电压
 * @param ang 电角度正余弦
 * @param out alpha beta电压
 */
void foc_inv_park(const foc_qd_current_t *in, const foc_sin_cos_t *ang, foc_alpha_beta_volt_t *out);

/**
 * @brief clarke变换
 * 
 * @param in 三相电流
 * @param out alpha beta电流
 */
void foc_clarke(const foc_uvw_current_t *in, foc_alpha_beta_current_t *out);

#ifdef __cplusplus
}
#endif
