#include "app/MotorClass.h"
#include "hpm_clock_drv.h"
#include "project_config.h"
#include <foc/iir_filter.h>
#include <stdint.h>

#define EN_TIMER // 定义启用计算运行时间

#ifdef EN_TIMER
uint32_t time[4];
ATTR_PLACE_AT_NONCACHEABLE float xx_time[4];
#define GET_TIME(index) time[index] = read_csr(CSR_CYCLE);
#else
#define GET_TIME(index)
#endif

ATTR_RAMFUNC void Motor_RunFoc(MotorClass_t *motor)
{
    foc_alpha_beta_current_t alpha_beta_current;
    foc_sin_cos_t sin_cos;
    foc_alpha_beta_volt_t alpha_beta_voltage;

    GET_TIME(0)
    if (motor->mode == DISABLE_MODE)
        return;

    GET_TIME(1);
    motor->raw_angle = motor->get_raw_angle_cb(motor); // 获取原始角度
    GET_TIME(2);
    motor->get_uvw_current_cb(motor, &motor->uvw_current); // 获取三相电流

    if (motor->mode == SVPWM_OPEN_LOOP_MODE)
    {
        /* svpwm开环模式使用angle_exp作为角度输入 */
        foc_sin_cos(motor->angle_exp * (2.0f * F_PI / ENCODER_MAX), &sin_cos);
    }
    else
    {
        /* 其他模式使用编码器作为角度输入 */
        encoder_get_eleAngle_sincos(&motor->encoder, motor->raw_angle, &sin_cos);
    }

#if SPEED_FILTER_MODE == SPEED_FILTER_IIR
    int16_t diff = foc_pid_diff(motor->raw_angle, motor->speed_pll.last_ang, ENCODER_MAX);
    motor->speed_pll.last_ang = motor->raw_angle;
    motor->speed_pll.speed = IIRFilter(&motor->speed_filter, (float)diff / ENCODER_MAX);
    motor->speed = motor->speed_pll.speed * 60 * PWM_FREQUENCY;
#endif

    /* 低速环 */
    if (++motor->intr_count >= (PWM_FREQUENCY / SPEED_PID_FREQUENCY))
    {
        motor->intr_count = 0;
#if SPEED_FILTER_MODE == SPEED_FILTER_PLL
        foc_pll(&motor->speed_pll, &sin_cos);
        motor->speed = motor->speed_pll.speed / motor->encoder.pole_pairs / (2 * F_PI) * 60 * SPEED_PID_FREQUENCY;
#elif SPEED_FILTER_MODE == SPEED_FILTER_PLL2
        foc_pll2(&motor->speed_pll, motor->raw_angle);
        motor->speed = motor->speed_pll.speed * 60 * SPEED_PID_FREQUENCY;
#endif
        if (motor->mode == ANGLE_MODE)
        {
            int diff = foc_pid_diff(motor->raw_angle, motor->angle_exp, ENCODER_MAX);
            motor->speed_exp = foc_pi_controller(&motor->angle_pid, diff, 0);
        }

        if (motor->mode >= SPEED_MODE)
        {
            motor->qd_current_exp.iq = foc_pi_controller(&motor->speed_pid, motor->speed, motor->speed_exp);
        }
    }
    motor->power = (motor->qd_voltage_exp.iq * motor->qd_current.iq + motor->qd_voltage_exp.id * motor->qd_current.id) *
                       motor->bus_voltage * 0.75f +
                   1.5f; // 估计值 0.75补偿系数，1.5静态功耗
    foc_clarke(&motor->uvw_current, &alpha_beta_current);
    foc_park(&alpha_beta_current, &sin_cos, &motor->qd_current);

    /* 电流环或更上层环计算 */
    if (motor->mode >= CURRENT_MODE)
    {
        motor->qd_voltage_exp.iq =
            foc_pi_controller(&motor->current_iq_pid, motor->qd_current.iq, motor->qd_current_exp.iq);
        motor->qd_voltage_exp.id =
            foc_pi_controller(&motor->current_id_pid, motor->qd_current.id, motor->qd_current_exp.id);
    }

    foc_inv_park(&motor->qd_voltage_exp, &sin_cos, &alpha_beta_voltage);
    foc_svpwm(&alpha_beta_voltage, &motor->pwm);
    motor->set_pwm_cb(motor, &motor->pwm);
    GET_TIME(3);

#ifdef EN_TIMER
    float tmp = clock_get_frequency(clock_cpu0) / 1e6f;
    xx_time[0] = (time[1] - time[0]) / tmp; // 进入耗时
    xx_time[1] = (time[2] - time[1]) / tmp; // 编码器数耗时
    xx_time[2] = (time[3] - time[2]) / tmp; // 三角函&环耗时
    xx_time[3] = (time[3] - time[0]) / tmp; // 总耗时
#endif
}

void Motor_Init(MotorClass_t *motor)
{
    motor->current_iq_pid.output_limit = UQ_LIMIT;
    motor->current_id_pid.output_limit = UD_LIMIT;
    motor->intr_count = 0;
    foc_pid_init(&motor->angle_pid);
    foc_pid_init(&motor->speed_pid);
    foc_pid_init(&motor->current_iq_pid);
    foc_pid_init(&motor->current_id_pid);
    foc_pll_init(&motor->speed_pll);
}

void Motor_SetMode(MotorClass_t *motor, MotorMode mode)
{
    if (mode == DISABLE_MODE && motor->mode != DISABLE_MODE)
        motor->enable_pwm(motor, false);

    if (mode != DISABLE_MODE && motor->mode == DISABLE_MODE)
        motor->enable_pwm(motor, true);

    motor->mode = mode;
}
