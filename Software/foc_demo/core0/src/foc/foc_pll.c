#include "foc/foc_pll.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "foc/foc_pid.h"
#include "project_config.h"
#include <stdint.h>

void foc_pll_init(foc_pll_t *pll)
{
    pll->speed = 0;
    pll->_epsilon = 0;
    pll->theta = 0;
    foc_pid_init(&pll->pi);
}

void foc_pll(foc_pll_t *pll, const foc_sin_cos_t *input)
{
    foc_sin_cos_t theta_sin_cos;
    foc_sin_cos(pll->theta, &theta_sin_cos);
    pll->_epsilon = input->sinx * theta_sin_cos.cosx - input->cosx * theta_sin_cos.sinx;
    pll->speed = foc_pi_controller(&pll->pi, pll->_epsilon, 0);
    pll->theta += pll->speed;

    /* 防溢出 */
    while (pll->theta > F_PI)
        pll->theta -= 2 * F_PI;

    while (pll->theta < -F_PI)
        pll->theta += 2 * F_PI;
}

void foc_pll2(foc_pll_t *pll, uint16_t raw_ang)
{    
    int16_t diff = foc_pid_diff(raw_ang, pll->last_ang, 65536);
    pll->last_ang = raw_ang;
    pll->speed += (diff / 65536.0f - pll->speed) * pll->pi.kp;
    pll->speed = foc_pid_limit(pll->speed, pll->pi.output_limit);
}