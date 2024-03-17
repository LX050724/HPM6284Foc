#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/MotorClass.h"
#include "app/current_calibration/current_calibration.h"
#include "app/electrical_angle_calibration/electrical_angle_calibration.h"
#include "board.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "hardware/adc_init.h"
#include "hardware/drv832x.h"
#include "hardware/encoder/encoder.h"
#include "hardware/encoder/mt6701_spi.h"
#include "hardware/hrpwm.h"
#include "hardware/trgm.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "multicore_common.h"
#include "project_config.h"
#include "stdbool.h"

static void motor0_get_uvw_current(MotorClass_t *motor, foc_uvw_current_t *uvw_current);
static uint16_t motor0_get_raw_angle(MotorClass_t *motor);
static void motor0_set_pwm(MotorClass_t *motor, const foc_pwm_t *pwm);
static void motor0_enable_pwm(MotorClass_t *motor, bool en);

/**
 * @brief
 *
 * @param _R1       当前温度下的电阻
 * @param _B        所使用的NTC电阻B值(datasheet里面有,例如3950)
 * @param _R2       T2温度下的电阻
 * @param _T2       一般是25℃
 * @return float    返回的就是当前温度(℃)
 */
float resistanceToTemperature(float _R1)
{
    float _B = 3950;
    float _R2 = 10e3;
    float _T2 = 25;
    return (1.0f / ((1.0f / _B) * log(_R1 / _R2) + (1.0f / (_T2 + 273.15f))) - 273.15f);
}

static void adc_callback(ADC16_Type *adc, uint32_t flag);

DMA_ATTR MotorClass_t motor0;

int main(void)
{
    board_init();

    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);

    drv832x_init_pins();
    drv832x_init_spi();
    drv832x_enable(true);
    clock_cpu_delay_ms(5);

    drv832x_dump_regs();
    drv832x_unlock();
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERP_HS, IDRIVEP_1000mA);
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERN_HS, IDRIVEN_1360mA);
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_LS_IDRIVERP_LS, IDRIVEP_570mA);
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_LS_IDRIVERN_LS, IDRIVEN_1360mA);
    drv832x_set(DRV_REG_OCP_CTL_DEAD_TIME, OCP_DEAD_TIME_50ns);

#if CURRENT_AMP == 40
    drv832x_set(DRV_REG_CSA_CTL_CSA_GAIN, CSA_GAIN_40);
#elif CURRENT_AMP == 20
    drv832x_set(DRV_REG_CSA_CTL_CSA_GAIN, CSA_GAIN_20);
#elif CURRENT_AMP == 10
    drv832x_set(DRV_REG_CSA_CTL_CSA_GAIN, CSA_GAIN_10);
#elif CURRENT_AMP == 5
    drv832x_set(DRV_REG_CSA_CTL_CSA_GAIN, CSA_GAIN_5);
#else
#error "CURRENT_AMP must is 40/20/10/5"
#endif

    drv832x_lock();
    drv832x_dump_regs();

    motor0.speed_pll.pi.output_limit = 2000;

    motor0.speed_pll.pi.kp = 0.05f;
    motor0.speed_pll.pi.output_limit = 200;

    motor0.angle_pid.kp = 0.08f;
    motor0.angle_pid.ki = 0.002f;
    motor0.angle_pid.integral_limit = 300;
    motor0.angle_pid.output_limit = 1000;

    motor0.speed_pid.kp = 0.01f;
    motor0.speed_pid.ki = 0.01f;
    motor0.speed_pid.integral_limit = 5;
    motor0.speed_pid.output_limit = 10;

    motor0.current_iq_pid.kp = 0.5f;
    motor0.current_iq_pid.ki = 0.0f;
    motor0.current_iq_pid.integral_limit = 6;
    motor0.current_id_pid.kp = 1.6f;
    motor0.current_id_pid.ki = 0.03f;
    motor0.current_id_pid.integral_limit = 3;

    motor0.get_uvw_current_cb = motor0_get_uvw_current;
    motor0.get_raw_angle_cb = motor0_get_raw_angle;
    motor0.set_pwm_cb = motor0_set_pwm;
    motor0.enable_pwm = motor0_enable_pwm;

    Motor_Init(&motor0);

    /* PWM初始化 */
    pwm_init(1, 0);

    /* ADC初始化 */
    adc_init();
    adc_enable_irq(1);
    adc_enable_it();

    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, trgm_output_same_as_input, false);

    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX0, trgm_output_same_as_input, false);
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_TRGM1_P_05;
    HPM_IOC->PAD[IOC_PAD_PA25].PAD_CTL = IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_DS_SET(7);
    trgm_connect(HPM_TRGM1, HPM_TRGM1_INPUT_SRC_TRGM0_OUTX0, HPM_TRGM1_OUTPUT_SRC_TRGM1_P5, trgm_output_pulse_at_input_rising_edge, false);
    trgm_enable_io_output(HPM_TRGM1, 1 << 5);

    /* adc中值校准程序 */
    // drv832x_calibration(true);
    clock_cpu_delay_ms(1);
    current_calibration(&motor0);
    // drv832x_calibration(false);

#if ENCODER_TYPE == ENCODER_MT6701
    mt6701_spi_init();
#endif

    motor0.encoder.ang_offset = 65197;
    motor0.encoder.pole_pairs = -7;
    // if (electrical_angle_calibration(&motor0) == 0)
    {
        motor0.qd_current_exp.iq = 1;
        // motor0.qd_voltage_exp.iq = 0.8;
        Motor_SetMode(&motor0,     CURRENT_MODE);
        adc_enable_irq(1);
        adc_enable_it();
        adc_set_callback(adc_callback);
    }

    while (1)
    {
        // motor0.angle_exp += 100;
        // if (motor0.angle_exp >= 65536.0f)
        //     motor0.angle_exp = 0;
        // clock_cpu_delay_ms(1);
    }
    return 0;
}

static void adc_callback(ADC16_Type *adc, uint32_t flag)
{
    Motor_RunFoc(&motor0);
}

static void motor0_get_uvw_current(MotorClass_t *motor, foc_uvw_current_t *uvw_current)
{
    uint16_t cal[3];
    adc_get_trigger0a_raw(cal);
    current_get_cal(&motor->current_cal, cal, uvw_current);
}

static uint16_t motor0_get_raw_angle(MotorClass_t *motor)
{
    return encoder_get_rawAngle();
}

static void motor0_set_pwm(MotorClass_t *motor, const foc_pwm_t *pwm)
{
    pwm_setvalue(pwm);
}

static void motor0_enable_pwm(MotorClass_t *motor, bool en)
{
    en ? pwm_enable_all_output() : pwm_disable_all_output();
}