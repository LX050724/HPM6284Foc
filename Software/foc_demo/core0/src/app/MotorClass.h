#pragma once

#include "foc/foc_core.h"
#include "foc/foc_pid.h"
#include "foc/foc_pll.h"
#include "foc/iir_filter.h"
#include "hardware/current/current.h"
#include "hardware/encoder/encoder.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MotorMode
{
    DISABLE_MODE,
    SVPWM_OPEN_LOOP_MODE,
    VOLTAGE_OPEN_LOOP_MODE,
    CURRENT_MODE,
    SPEED_MODE,
    ANGLE_MODE
} MotorMode;

typedef struct MotorClass_t
{
    int motor_id;
    MotorMode mode;

    int intr_count;

    encoder_t encoder;
    CurrentCal_t current_cal;

    foc_pll_t speed_pll;
#if SPEED_FILTER_MODE == SPEED_FILTER_IIR
    IIRFilter_t speed_filter;
#endif

    foc_pid_contrl_t angle_pid;
    foc_pid_contrl_t speed_pid;
    foc_pid_contrl_t current_iq_pid;
    foc_pid_contrl_t current_id_pid;

    float bus_voltage;
    float power;

    uint16_t angle_exp;
    float speed_exp;
    foc_qd_current_t qd_current_exp;
    foc_qd_current_t qd_voltage_exp;

    foc_uvw_current_t uvw_current;
    foc_qd_current_t qd_current;
    foc_pwm_t pwm;

    float speed;
    uint16_t raw_angle;

    void (*get_uvw_current_cb)(struct MotorClass_t *, foc_uvw_current_t *);
    uint16_t (*get_raw_angle_cb)(struct MotorClass_t *);
    void (*set_pwm_cb)(struct MotorClass_t *, const foc_pwm_t *);
    void (*enable_pwm)(struct MotorClass_t *, bool);
} MotorClass_t;

void Motor_Init(MotorClass_t *motor);
void Motor_RunFoc(MotorClass_t *motor);
void Motor_SetMode(MotorClass_t *motor, enum MotorMode mode);

#ifdef __cplusplus
}
#endif