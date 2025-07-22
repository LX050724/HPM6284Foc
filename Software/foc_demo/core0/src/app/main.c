#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/MotorClass.h"
#include "app/current_calibration/current_calibration.h"
#include "app/electrical_angle_calibration/electrical_angle_calibration.h"
#include "board.h"
#include "foc/foc_core.h"
#include "hardware/adc_init.h"
#include "hardware/drv832x.h"
#include "hardware/encoder/encoder.h"
#include "hardware/encoder/mt6701_spi.h"
#include "hardware/hrpwm.h"
#include "hardware/trgm.h"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_ioc_regs.h"
#include "hpm_iomux.h"
#include "hpm_mbx_drv.h"
#include "hpm_misc.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "multicore_common.h"
#include "project_config.h"
#include "stdbool.h"
#include <core_comm.h>

#define MAIN_DEBUG(fmt, ...) DLOG("MAIN", fmt, ##__VA_ARGS__)

static void motor0_get_uvw_current(MotorClass_t *motor, foc_uvw_current_t *uvw_current);
static uint16_t motor0_get_raw_angle(MotorClass_t *motor);
static void motor0_set_pwm(MotorClass_t *motor, const foc_pwm_t *pwm);
static void motor0_enable_pwm(MotorClass_t *motor, bool en);

ATTR_PLACE_AT_FAST_RAM_BSS MotorClass_t motor0;
ATTR_PLACE_AT_FAST_RAM_BSS volatile int vofa_write_ptr;

volatile ATTR_SHARE_MEM core_comm_ctl_t core_comm_ctl;

#if SPEED_FILTER_MODE == SPEED_FILTER_IIR
/**
 * @brief IIR滤波器系数，切比雪夫II型@100Khz低通滤波器，通带频率100Hz/200Hz
 */
const float FLITER_NUM[][3] = {{0.3588240147, 0, 0},
                               {1, -1.999825358, 1},
                               {0.1754118353, 0, 0},
                               {1, -1.999542952, 1},
                               {0.004925392102, 0, 0},
                               {1, 1, 0},
                               {1, 0, 0}};
const float FLITER_DEN[][3] = {{1, 0, 0}, {1, -1.996026397, 0.9960891008},
                               {1, 0, 0}, {1, -1.986816645, 0.9868968725},
                               {1, 0, 0}, {1, -0.9901492, 0},
                               {1, 0, 0}};
#endif

static void adc_callback(ADC16_Type *adc, uint32_t flag);

int main(void)
{
    board_init();
    memset(&core_comm_ctl, 0, sizeof(core_comm_ctl));
    for (int i = 0; i < BUF_NUM; i++)
    {
        core_comm_ctl.vofa_buf[i].tail[0] = 0x00;
        core_comm_ctl.vofa_buf[i].tail[1] = 0x00;
        core_comm_ctl.vofa_buf[i].tail[2] = 0x80;
        core_comm_ctl.vofa_buf[i].tail[3] = 0x7f;
    }

    mbx_init(HPM_MBX0A);
    intc_m_enable_irq_with_priority(IRQn_MBX0A, 1);
    mbx_enable_intr(HPM_MBX0A, MBX_CR_RWMVIE_MASK);

    /* 启动CPU1并等待返回启动信息 */
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);

    drv832x_init_pins();
    drv832x_init_spi();
    drv832x_enable(true);
    clock_cpu_delay_ms(5);

    drv832x_dump_regs();
    drv832x_unlock();
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERP_HS, IDRIVEP_1000mA);
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERN_HS, IDRIVEN_1360mA);
    drv832x_set(DRV_REG_DRIVER_GATE_DRIVER_LS_IDRIVERP_LS, IDRIVEP_1000mA);
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

    motor0.current_iq_pid.kp = 0.4f;
    motor0.current_iq_pid.ki = 0.06f;
    motor0.current_iq_pid.integral_limit = 6;
    motor0.current_id_pid.kp = 1.6f;
    motor0.current_id_pid.ki = 0.03f;
    motor0.current_id_pid.integral_limit = 3;

    motor0.get_uvw_current_cb = motor0_get_uvw_current;
    motor0.get_raw_angle_cb = motor0_get_raw_angle;
    motor0.set_pwm_cb = motor0_set_pwm;
    motor0.enable_pwm = motor0_enable_pwm;
#if SPEED_FILTER_MODE == SPEED_FILTER_IIR
    IIRFilterInit(&motor0.speed_filter, 3, FLITER_NUM, FLITER_DEN);
#endif
    Motor_Init(&motor0);

    /* 按键通过TRGM1输出0连接到PWM0内部故障输入1 */
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_TRGM1_P_07;
    HPM_IOC->PAD[IOC_PAD_PA27].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    trgm_input_filter_t filter = {};
    filter.mode = trgm_filter_mode_rapid_change;
    filter.sync = true;
    filter.invert = false;
    filter.filter_length = TRGM_FILTCFG_FILTLEN_MASK;
    trgm_input_filter_config(HPM_TRGM1, HPM_TRGM1_FILTER_SRC_TRGM1_IN7, &filter);
    trgm_connect(HPM_TRGM1, HPM_TRGM1_INPUT_SRC_TRGM1_P7, HPM_TRGM1_OUTPUT_SRC_TRGM1_OUTX0, trgm_output_same_as_input,
                 true);
    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_TRGM1_OUTX0, HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI1,
                 trgm_output_same_as_input, false);

    /* 连接DRV故障线到PWM0内部故障输入0并取反 */
    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_TRGM0_P6, HPM_TRGM0_OUTPUT_SRC_PWM0_FAULTI0, trgm_output_same_as_input,
                 true);

    /* PWM0采样时刻通过TRGM0输出0连接到PA25 */
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_TRGM1_P_05;
    HPM_IOC->PAD[IOC_PAD_PA25].PAD_CTL = IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_DS_SET(7);
    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_PWM0_CH15REF, HPM_TRGM0_OUTPUT_SRC_TRGM0_OUTX0,
                 trgm_output_same_as_input, false);
    trgm_connect(HPM_TRGM1, HPM_TRGM1_INPUT_SRC_TRGM0_OUTX0, HPM_TRGM1_OUTPUT_SRC_TRGM1_P5,
                 trgm_output_pulse_at_input_rising_edge, false);
    trgm_enable_io_output(HPM_TRGM1, 1 << 5);

    /* PWM初始化 */
    pwm_init(1, 0);

    /* ADC初始化 */
    adc_init();
    adc_enable_irq(1);
    adc_enable_it();

    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A,
                 trgm_output_same_as_input, false);

    /* adc中值校准程序 */
    // drv832x_calibration(true);
    // clock_cpu_delay_ms(1);
    current_calibration(&motor0);
    // drv832x_calibration(false);

#if ENCODER_TYPE == ENCODER_MT6701
    mt6701_spi_init();
#endif

    // motor0.encoder.ang_offset = 0x81b6;
    // motor0.encoder.pole_pairs = -7;
    if (electrical_angle_calibration(&motor0) == 0)
    { 
        motor0.qd_current_exp.iq = 5;
        motor0.qd_voltage_exp.iq = 0.8;
        Motor_SetMode(&motor0, CURRENT_MODE);
        adc_enable_irq(2);
        adc_enable_it();
        adc_set_callback(adc_callback);
    }

    while (1)
    {
        if (motor0.mode == SVPWM_OPEN_LOOP_MODE)
        {
            motor0.angle_exp += 100;
            clock_cpu_delay_us(200);
        }
    }
    return 0;
}

static int count = 0;
static void adc_callback(ADC16_Type *adc, uint32_t flag)
{
    Motor_RunFoc(&motor0);

    just_float_data *pdata = &core_comm_ctl.vofa_buf[vofa_write_ptr];
    pdata->data[0] = motor0.qd_current.iq;
    pdata->data[1] = motor0.qd_current_exp.iq;
    pdata->data[2] = motor0.qd_current.id;
    pdata->data[3] = motor0.qd_current_exp.id;
    pdata->data[4] = motor0.speed;
    pdata->data[5] = motor0.speed_exp;
    pdata->data[6] = motor0.uvw_current.iu;
    pdata->data[7] = motor0.uvw_current.iv;
    pdata->data[8] = motor0.uvw_current.iw;
    pdata->data[9] = motor0.raw_angle;
    pdata->data[10] = motor0.bus_voltage;
    pdata->data[11] = motor0.power;
    pdata->data[12] = count;

    if (++count > 1000)
        count = 0;

    if (vofa_write_ptr == 0)
    {
        mbx_send_message(HPM_MBX0A, CORE0_VOFA_UPLOAD_BUF2);
    }

    if (vofa_write_ptr == BUF_NUM / 2)
    {
        mbx_send_message(HPM_MBX0A, CORE0_VOFA_UPLOAD_BUF1);
    }
    vofa_write_ptr = (vofa_write_ptr + 1);
    if (vofa_write_ptr == BUF_NUM)
        vofa_write_ptr = 0;
}

SDK_DECLARE_EXT_ISR_M(IRQn_MBX0A, isr_mbx)
void isr_mbx(void)
{
    volatile uint32_t sr = HPM_MBX0A->SR;
    volatile uint32_t cr = HPM_MBX0A->CR;
    uint32_t msg = 0;

    if (!((sr & MBX_SR_RWMV_MASK) && (cr & MBX_CR_RWMVIE_MASK)))
        return;

    mbx_retrieve_message(HPM_MBX0A, &msg);

    if (msg == CORE1_CONTROL_MSG)
    {
#define CONV_DATA(NAME, DST)                                                                                           \
    case offsetof(SetData_t, NAME):                                                                                    \
        (DST) = core_comm_ctl.set_data.NAME;                                                                          \
        break;

        switch (core_comm_ctl.set_data.set_offset)
        {
            CONV_DATA(exp_iq, motor0.qd_current_exp.iq);
            CONV_DATA(exp_id, motor0.qd_current_exp.id);
            CONV_DATA(id_p, motor0.current_id_pid.kp);
            CONV_DATA(id_i, motor0.current_id_pid.ki);
            CONV_DATA(id_il, motor0.current_id_pid.integral_limit);
            CONV_DATA(iq_p, motor0.current_iq_pid.kp);
            CONV_DATA(iq_i, motor0.current_iq_pid.ki);
            CONV_DATA(iq_il, motor0.current_iq_pid.integral_limit);
            CONV_DATA(speed_p, motor0.speed_pid.kp);
            CONV_DATA(speed_i, motor0.speed_pid.ki);
            CONV_DATA(exp_speed, motor0.speed_exp);
            CONV_DATA(ang_p, motor0.angle_pid.kp);
            CONV_DATA(ang_i, motor0.angle_pid.ki);
            CONV_DATA(exp_ang, motor0.angle_exp);
            CONV_DATA(speed_filter, motor0.speed_pll.pi.kp);
        default:
            break;
        }
        return;
    }
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