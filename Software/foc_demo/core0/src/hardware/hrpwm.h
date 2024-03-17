#pragma once

#include "board.h"
#include "foc/foc_core.h"
#include "hpm_pwm_drv.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int pwm_init(uint32_t ch8_cmp, uint32_t dead_time_ns);

static inline void pwm_enable_all_output(void)
{
    pwm_disable_sw_force(BOARD_BLDCPWM);
}

static inline void pwm_disable_all_output(void)
{
    pwm_enable_sw_force(BOARD_BLDCPWM);
}

void pwm_setvalue(const foc_pwm_t *par);

#ifdef __cplusplus
}
#endif