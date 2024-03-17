#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_interrupt.h"
#include "hpm_mbx_drv.h"
#include "hpm_misc.h"
#include "hpm_soc.h"
#include "usb_dc.h"
#include <core_comm.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <usb_config.h>

extern volatile uint8_t dtr_enable;
extern volatile uint8_t rts_enable;
extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
extern volatile bool ep_tx_busy_flag;

void vofa_push_send(MsgID buf_index);

volatile ATTR_SHARE_MEM core_comm_ctl_t core_comm_ctl;

int main(void)
{
    clock_update_core_clock();
    l1c_dc_disable();

    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_GPIO_B_11;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 11, gpiom_soc_gpio1);
    gpiom_disable_pin_visibility(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 11, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO1, GPIO_OE_GPIOB, 11);
    gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 1);

    clock_add_to_group(clock_usb0, 1);
    cdc_acm_init();
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 2);

    mbx_init(HPM_MBX0B);
    intc_m_enable_irq_with_priority(IRQn_MBX0B, 1);
    mbx_enable_intr(HPM_MBX0B, MBX_CR_RWMVIE_MASK);

    /* 发送CPU1的vofa输入缓冲区内存地址到CPU0 */
    uint32_t addr = core_local_mem_to_sys_address(HPM_CORE1, (uint32_t)&core_comm_ctl);
    mbx_send_message(HPM_MBX0B, addr);

    while (1)
    {
        gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 0);
        clock_cpu_delay_ms(300);
        gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 1);
        clock_cpu_delay_ms(300);
    }
    return 0;
}

SDK_DECLARE_EXT_ISR_M(IRQn_MBX0B, isr_mbx)
void isr_mbx(void)
{
    volatile uint32_t sr = HPM_MBX0B->SR;
    volatile uint32_t cr = HPM_MBX0B->CR;

    if (!((sr & MBX_SR_RWMV_MASK) && (cr & MBX_CR_RWMVIE_MASK)))
        return;

    MsgID msg_id = INVALID_MSG;
    mbx_retrieve_message(HPM_MBX0B, (uint32_t *)&msg_id);
    switch (msg_id)
    {
    case CORE0_VOFA_UPLOAD_BUF1:
    case CORE0_VOFA_UPLOAD_BUF2:
        vofa_push_send(msg_id);
        break;
    case CORE0_UPDATAE_CTL:
        DCache_invalidate((void *)&core_comm_ctl, sizeof(core_comm_ctl));
        break;
    default:
        break;
    }
}

typedef struct
{
    const char *name;
    volatile float *tar_val;
} CmdCallback_t;

CmdCallback_t cmd_list[] = {
    {"exp_iq", &core_comm_ctl.set_data.exp_iq},
    {"exp_id", &core_comm_ctl.set_data.exp_id},
    {"id_p", &core_comm_ctl.set_data.id_p},
    {"id_i", &core_comm_ctl.set_data.id_i},
    {"id_il", &core_comm_ctl.set_data.id_il},
    {"iq_p", &core_comm_ctl.set_data.iq_p},
    {"iq_i", &core_comm_ctl.set_data.iq_i},
    {"iq_il", &core_comm_ctl.set_data.iq_il},
    {"speed_p", &core_comm_ctl.set_data.speed_p},
    {"speed_i", &core_comm_ctl.set_data.speed_i},
    {"exp_speed", &core_comm_ctl.set_data.exp_speed},
    {"ang_p", &core_comm_ctl.set_data.ang_p},
    {"ang_i", &core_comm_ctl.set_data.ang_i},
    {"exp_ang", &core_comm_ctl.set_data.exp_ang},
    {"speed_filter", &core_comm_ctl.set_data.speed_filter},
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
                    /* 写入数据，设置更改地址偏移量，发送消息 */
                    *(cmd_list[index].tar_val) = value;
                    core_comm_ctl.set_data.set_offset =
                        (uint32_t)(cmd_list[index].tar_val) - (uint32_t)&core_comm_ctl.set_data;
                    DCache_flush((void *)&core_comm_ctl.set_data, sizeof(SetData_t));
                    mbx_send_message(HPM_MBX0B, CORE1_CONTROL_MSG);
                    break;
                }
            }
        }
    }
}

void vofa_push_send(MsgID buf_index)
{
    if (!dtr_enable)
        return;

    if (buf_index == CORE0_VOFA_UPLOAD_BUF1)
    {
        if (!ep_tx_busy_flag)
        {
            ep_tx_busy_flag = true;
            usbd_ep_start_write(0x81, (uint8_t *)core_comm_ctl.vofa_buf1, sizeof(just_float_data) * BUF_NUM / 2);
        }
    }
    else if (buf_index == CORE0_VOFA_UPLOAD_BUF2)
    {
        if (!ep_tx_busy_flag)
        {
            ep_tx_busy_flag = true;
            usbd_ep_start_write(0x81, (uint8_t *)core_comm_ctl.vofa_buf2, sizeof(just_float_data) * BUF_NUM / 2);
        }
    }
}