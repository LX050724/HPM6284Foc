#include "hrpwm.h"
#include "board.h"
#include "foc/foc_core.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_pwm_drv.h"
#include "hpm_pwm_regs.h"
#include "pinmux.h"
#include "project_config.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>

#define DEAD_TIME_NS(t) ((uint32_t)((t) / 1e9f * AHB_CLOCK * 2)) // 计算t ns的死区时间值

/**
 * @brief 复位PWM计数器
 *
 */
static inline void reset_pwm_counter(void)
{
    pwm_enable_reload_at_synci(BOARD_BLDCPWM);
}

int pwm_init(uint32_t adc_trigger_cmp, uint32_t dead_time_ns)
{
    pwm_cmp_config_t cmp_config[4] = {};
    pwm_pair_config_t pwm_pair_config = {};
    pwm_output_channel_t pwm_output_ch_cfg = {};

    init_pwm_pins(BOARD_BLDCPWM);

    pwm_deinit(BOARD_BLDCPWM);
    pwm_stop_counter(BOARD_BLDCPWM);
    reset_pwm_counter();

#if HRPWM_ENABLE
    pwm_cal_hrpwm_start(BOARD_BLDCPWM);
    for (int chn = 0; chn < 8; chn++)
        pwm_cal_hrpwm_chn_wait(BOARD_BLDCPWM, chn);

    pwm_enable_hrpwm(BOARD_BLDCPWM);
    pwm_set_hrpwm_reload(BOARD_BLDCPWM, HRPWM_RELOAD, PWM_RELOAD);
    pwm_set_hrpwm_start_count(BOARD_BLDCPWM, 0);
#else
    pwm_set_reload(BOARD_BLDCPWM, 0, PWM_RELOAD);
    pwm_set_start_count(BOARD_BLDCPWM, 0, 0);
#endif

    /* 配置PWM置位比较寄存器 */
    cmp_config[0].enable_hrcmp = !!(HRPWM_ENABLE);
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = PWM_RELOAD + 1;
    cmp_config[0].hrcmp = HRPWM_RELOAD;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    /* 配置PWM复位比较寄存器 */
    cmp_config[1].enable_hrcmp = !!(HRPWM_ENABLE);
    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = PWM_RELOAD + 1;
    cmp_config[1].hrcmp = HRPWM_RELOAD;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    pwm_get_default_pwm_pair_config(BOARD_BLDCPWM, &pwm_pair_config);

    pwm_pair_config.pwm[0].hrpwm_update_mode = !!(HRPWM_ENABLE);
    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = DEAD_TIME_NS(dead_time_ns);
    pwm_pair_config.pwm[0].invert_output = false;
    pwm_pair_config.pwm[0].fault_mode = pwm_fault_mode_force_output_0;            // 设置故障保护电平
    pwm_pair_config.pwm[0].fault_recovery_trigger = pwm_fault_recovery_on_reload; // 故障清除后下一个PWM周期自动恢复
    pwm_pair_config.pwm[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    pwm_pair_config.pwm[1].hrpwm_update_mode = !!(HRPWM_ENABLE);
    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = DEAD_TIME_NS(dead_time_ns);
    pwm_pair_config.pwm[1].invert_output = false;
    pwm_pair_config.pwm[1].fault_mode = pwm_fault_mode_force_output_0;            // 设置故障保护电平
    pwm_pair_config.pwm[1].fault_recovery_trigger = pwm_fault_recovery_on_reload; // 故障清除后下一个PWM周期自动恢复
    pwm_pair_config.pwm[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    /*
     * config pwm
     */
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, &pwm_pair_config,
                                                     BOARD_BLDCPWM_CMP_INDEX_0, &cmp_config[0], 2))
    {
        // SEGGER_RTT_printf(0, "failed to setup waveform U\n");
        return 1;
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, &pwm_pair_config,
                                                     BOARD_BLDCPWM_CMP_INDEX_2, &cmp_config[0], 2))
    {
        // SEGGER_RTT_printf(0, "failed to setup waveform V\n");
        return 1;
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, &pwm_pair_config,
                                                     BOARD_BLDCPWM_CMP_INDEX_4, &cmp_config[0], 2))
    {
        // SEGGER_RTT_printf(0, "failed to setup waveform W\n");
        return 1;
    }

    /* 配置边缘采样时刻比较寄存器 */
    cmp_config[2].enable_hrcmp = !!(HRPWM_ENABLE);
    cmp_config[2].enable_ex_cmp = false;
    cmp_config[2].mode = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = adc_trigger_cmp;
    cmp_config[2].hrcmp = 0;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_hw_event;
    pwm_config_cmp(BOARD_BLDCPWM, 8, &cmp_config[2]);

    pwm_output_ch_cfg.cmp_start_index = 8; /* start channel 8 */
    pwm_output_ch_cfg.cmp_end_index = 8;   /* end channel 8 */
    pwm_output_ch_cfg.invert_output = false;
    pwm_config_output_channel(BOARD_BLDCPWM, 8, &pwm_output_ch_cfg);

    /* 配置寄存器加载时刻比较寄存器 */
    cmp_config[3].enable_hrcmp = !!(HRPWM_ENABLE);
    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = PWM_RELOAD;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;
    pwm_load_cmp_shadow_on_match(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, &cmp_config[3]);

    /* 开启debug故障保护模式 */
    pwm_fault_source_config_t fault_config = {};
    fault_config.source_mask = PWM_GCR_DEBUGFAULT_SET(1) | PWM_GCR_FAULTI1EN_SET(1) | PWM_GCR_FAULTI0EN_SET(1);
    pwm_config_fault_source(BOARD_BLDCPWM, &fault_config);

    /* 初始化软件强制输出状态，并默认关闭所有输出 */
    pwm_config_force_cmd_timing(BOARD_BLDCPWM, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_BLDCPWM, PWM_FORCE_OUTPUT(BOARD_BLDC_UH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_UL_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_VH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_VL_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_WH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_WL_PWM_OUTPIN, pwm_output_0));

    pwm_disable_all_output();

    pwm_enable_irq(BOARD_BLDCPWM, PWM_IRQEN_FAULTIRQE_MASK);
    intc_m_enable_irq_with_priority(IRQn_PWM0, 3);

    pwm_start_counter(BOARD_BLDCPWM);
    pwm_issue_shadow_register_lock_event(BOARD_BLDCPWM);
    return 0;
}

#if HRPWM_ENABLE
void pwm_setvalue(const foc_pwm_t *par)
{
    if (par->pwm_u >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_u >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_MAX / 2 - pwm_half);
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_MAX / 2 + pwm_half);
    }

    if (par->pwm_v >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_v >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_MAX / 2 - pwm_half);
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_MAX / 2 + pwm_half);
    }

    if (par->pwm_w >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_CMP_HRPWM_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_w >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_MAX / 2 - pwm_half);
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_MAX / 2 + pwm_half);
    }
}
#else
void pwm_setvalue(const foc_pwm_t *par)
{
    if (par->pwm_u >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_CMP_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_CMP_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_u >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) + pwm_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) - pwm_half)));
    }

    if (par->pwm_v >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_CMP_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_CMP_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_v >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) + pwm_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) - pwm_half)));
    }

    if (par->pwm_w >= PWM_MAX)
    {
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_CMP_CMP_SET(PWM_RELOAD));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_CMP_CMP_SET(PWM_RELOAD + 1));
    }
    else
    {
        uint32_t pwm_half = par->pwm_w >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) + pwm_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_CMP_CMP_SET(((PWM_RELOAD / 2) - pwm_half)));
    }
}
#endif

static SDK_DECLARE_EXT_ISR_M(IRQn_PWM0, __isr_pwm0_fun);
static void __isr_pwm0_fun(void)
{
    pwm_clear_status(BOARD_BLDCPWM, PWM_IRQEN_FAULTIRQE_MASK);
    // pwm_clear_fault(BOARD_BLDCPWM);
}