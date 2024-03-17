#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_mbx_drv.h"
#include <stdio.h>
#include <usb_config.h>

extern volatile uint8_t dtr_enable;
extern volatile uint8_t rts_enable;
extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
extern volatile bool ep_tx_busy_flag;

int main(void)
{
    clock_update_core_clock();

    clock_add_to_group(clock_usb0, 1);
    cdc_acm_init();
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 1);

    mbx_init(HPM_MBX0B);
    mbx_enable_intr(HPM_MBX0B, MBX_CR_RFMAIE_MASK);

    while (1)
    {
    }
    return 0;
}

SDK_DECLARE_EXT_ISR_M(IRQn_MBX0B, isr_mbx)
void isr_mbx(void)
{
    volatile uint32_t sr = HPM_MBX0B->SR;
    volatile uint32_t cr = HPM_MBX0B->CR;
    if ((sr & MBX_SR_RWMV_MASK) && (cr & MBX_CR_RWMVIE_MASK))
    {
        // mbx_disable_intr(MBX, MBX_CR_RWMVIE_MASK);
        // can_read = true;
    }
    if ((sr & MBX_SR_RFMA_MASK) && (cr & MBX_CR_RFMAIE_MASK))
    {
        // mbx_disable_intr(MBX, MBX_CR_RFMAIE_MASK);
        // can_read = true;
    }
    if ((sr & MBX_SR_TWME_MASK) && (cr & MBX_CR_TWMEIE_MASK))
    {
        // mbx_disable_intr(MBX, MBX_CR_TWMEIE_MASK);
        // can_send = true;
    }
    if ((sr & MBX_SR_TFMA_MASK) && (cr & MBX_CR_TFMAIE_MASK))
    {
        // mbx_disable_intr(MBX, MBX_CR_TFMAIE_MASK);
        // can_send = true;
    }
}

typedef struct
{
    const char *name;
    void (*fun)(float);
    float *tar_val;
} CmdCallback_t;

// void set_expang(float ang)
// {
//     motor0.angle_exp = ang;
// }

CmdCallback_t cmd_list[] = {
    // {"exp_iq", NULL, &motor0.qd_current_exp.iq},
    // {"exp_id", NULL, &motor0.qd_current_exp.id},
    // {"id_p", NULL, &motor0.current_id_pid.kp},
    // {"id_i", NULL, &motor0.current_id_pid.ki},
    // {"id_il", NULL, &motor0.current_id_pid.integral_limit},
    // {"iq_p", NULL, &motor0.current_iq_pid.kp},
    // {"iq_i", NULL, &motor0.current_iq_pid.ki},
    // {"iq_il", NULL, &motor0.current_iq_pid.integral_limit},
    // {"speed_p", NULL, &motor0.speed_pid.kp},
    // {"speed_i", NULL, &motor0.speed_pid.ki},
    // {"exp_speed", NULL, &motor0.speed_exp},
    // {"ang_p", NULL, &motor0.angle_pid.kp},
    // {"ang_i", NULL, &motor0.angle_pid.ki},
    // {"exp_ang", set_expang, NULL},
    // {"speed_filter", NULL, &motor0.speed_pll.pi.kp},
};

void usbd_read_callback(char *data, uint32_t len)
{
    char name[64] = {};
    float value = 0;
    char *start = data;
    for (int i = 0; i < len - 1; i++)
    {
        if (data[i] == ':')
        {
            strncpy(name, start, data + i - start);
            value = strtof(&data[i + 1], &start);
            if (start[0] == '\n')
                start++;
            for (int index = 0; index < sizeof(cmd_list) / sizeof(CmdCallback_t); index++)
            {
                if (strcmp(cmd_list[index].name, name) == 0)
                {
                    // SEGGER_RTT_printf(0, "set %s %d\n", cmd_list[index].name, (int)(value * 1000));
                    if (cmd_list[index].fun == NULL)
                    {
                        *(cmd_list[index].tar_val) = value;
                    }
                    else
                    {
                        cmd_list[index].fun(value);
                    }
                    break;
                }
            }
        }
    }
}