#include "foc_core.h"
#include "project_config.h"
#include <stdint.h>

#define SQRT3 1.732050807568877f
#define SQRT3_BY_2 (SQRT3 / 2.0f)

static inline int32_t foc_max(int32_t a, int32_t b)
{
    return a > b ? a : b;
}

static inline int32_t foc_min(int32_t a, int32_t b)
{
    return a < b ? a : b;
}

void foc_svpwm(const foc_alpha_beta_volt_t *volt, foc_pwm_t *pwm)
{
    int32_t pwm_half = PWM_RELOAD / 2;
    int32_t va = (volt->v_alpha) * pwm_half;
    int32_t vb = (volt->v_alpha * -0.5f + SQRT3_BY_2 * volt->v_beta) * pwm_half;
    int32_t vc = (volt->v_alpha * -0.5f - SQRT3_BY_2 * volt->v_beta) * pwm_half;
    int32_t vmax = foc_max(va, foc_max(vb, vc));
    int32_t vmin = foc_min(va, foc_min(vb, vc));
    int32_t vcom = (vmax + vmin) / 2 + pwm_half;

    pwm->pwm_u = vcom - va;
    pwm->pwm_v = vcom - vb;
    pwm->pwm_w = vcom - vc;
}

void foc_park(const foc_alpha_beta_current_t *in, const foc_sin_cos_t *ang, foc_qd_current_t *out)
{
    out->id = ang->cosx * in->i_alpha + ang->sinx * in->i_beat;
    out->iq = -ang->sinx * in->i_alpha + ang->cosx * in->i_beat;
}

void foc_inv_park(const foc_qd_current_t *in, const foc_sin_cos_t *ang, foc_alpha_beta_volt_t *out)
{
    out->v_alpha = ang->cosx * in->id - ang->sinx * in->iq;
    out->v_beta = ang->sinx * in->id + ang->cosx * in->iq;
}

void foc_clarke(const foc_uvw_current_t *in, foc_alpha_beta_current_t *out)
{
    out->i_alpha = in->iu;
    out->i_beat = 1 / SQRT3 * in->iu + 2 / SQRT3 * in->iv;
}
