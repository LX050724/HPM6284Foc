
#include "adc_init.h"
#include "board.h"
#include "hpm_adc.h"
#include "hpm_soc.h"
#include <stdint.h>

static adc_callback_t __isr_callback;

typedef struct
{
    uint32_t value : 16;
    uint32_t : 4;
    uint32_t channel : 4;
    uint32_t source : 5;
    uint32_t : 2;
    uint32_t flag : 1;
} AdcDMA_t;

static adc_type adc0;
static adc_type adc1;
static adc_type adc2;
static DMA_ATTR AdcDMA_t adc0_dma_buf[48];
static DMA_ATTR AdcDMA_t adc1_dma_buf[48];
static DMA_ATTR AdcDMA_t adc2_dma_buf[48];

void init_trigger_cfg(void)
{
}

void adc_init_pins()
{
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void adc_init()
{
    adc_config_t cfg;
    adc_channel_config_t ch_cfg;
    adc_pmt_config_t pmt_cfg;
    cfg.module = ADCX_MODULE_ADC16;

    adc0.module = adc_module_adc16,
    adc0.adc_base.adc16 = HPM_ADC0;
    adc1.module = adc_module_adc16,
    adc1.adc_base.adc16 = HPM_ADC1;
    adc2.module = adc_module_adc16,
    adc2.adc_base.adc16 = HPM_ADC2;

    board_init_adc16_pins();

    board_init_adc16_clock(HPM_ADC0, true);
    board_init_adc16_clock(HPM_ADC1, true);
    board_init_adc16_clock(HPM_ADC2, true);
    hpm_adc_init_default_config(&cfg);

    cfg.config.adc16.res = adc16_res_16_bits;
    cfg.config.adc16.conv_mode = adc16_conv_mode_preemption;
    cfg.config.adc16.adc_clk_div = adc16_clock_divider_4;
    cfg.config.adc16.sel_sync_ahb = false;
    cfg.config.adc16.adc_ahb_en = true;

    cfg.adc_base.adc16 = HPM_ADC0;
    hpm_adc_init(&cfg);
    cfg.adc_base.adc16 = HPM_ADC1;
    hpm_adc_init(&cfg);
    cfg.adc_base.adc16 = HPM_ADC2;
    hpm_adc_init(&cfg);

    ch_cfg.module = ADCX_MODULE_ADC16;
    hpm_adc_init_channel_default_config(&ch_cfg);

    /* 初始化ADC通道 */
    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_ADC_A_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_ADC_A_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_A;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_ADC_B_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_ADC_B_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_B;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_ADC_C_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_ADC_C_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_C;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_VOL_ADC_A_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_A_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_VOL_ADC_CH_A;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_VOL_ADC_B_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_B_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_VOL_ADC_CH_B;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_VOL_ADC_C_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_C_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_VOL_ADC_CH_C;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_BLDC_VOL_ADC_BUS_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_BUS_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_VOL_ADC_CH_BUS;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_NTC1_ADC_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_NTC1_ADC_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_NTC1_ADC_CH;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = BOARD_NTC2_ADC_SAMPLE_CLCYE;
    ch_cfg.adc_base.adc16 = BOARD_NTC2_ADC_BASE;
    ch_cfg.config.adc16_ch.ch = BOARD_NTC2_ADC_CH;
    hpm_adc_channel_init(&ch_cfg);

    /* 配置注入触发 */
    /* 电流采集 ADC16_CONFIG_TRG0A 触发 */
    pmt_cfg.module = ADCX_MODULE_ADC16;
    pmt_cfg.config.adc16.inten[0] = true;
    pmt_cfg.config.adc16.trig_ch = ADC16_CONFIG_TRG0A;

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_ADC_A_BASE;
    pmt_cfg.config.adc16.trig_len = 1;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_ADC_CH_A;
    hpm_adc_set_preempt_config(&pmt_cfg);

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_ADC_B_BASE;
    pmt_cfg.config.adc16.trig_len = 1;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_ADC_CH_B;
    hpm_adc_set_preempt_config(&pmt_cfg);

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_ADC_C_BASE;
    pmt_cfg.config.adc16.trig_len = 1;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_ADC_CH_C;
    hpm_adc_set_preempt_config(&pmt_cfg);

    adc16_set_pmt_queue_enable(BOARD_BLDC_ADC_A_BASE, ADC16_CONFIG_TRG0A, true);
    adc16_set_pmt_queue_enable(BOARD_BLDC_ADC_B_BASE, ADC16_CONFIG_TRG0A, true);
    adc16_set_pmt_queue_enable(BOARD_BLDC_ADC_C_BASE, ADC16_CONFIG_TRG0A, true);

    /* 电压采集 ADC16_CONFIG_TRG0B 触发 */
    pmt_cfg.module = ADCX_MODULE_ADC16;
    pmt_cfg.config.adc16.inten[0] = true;
    pmt_cfg.config.adc16.trig_ch = ADC16_CONFIG_TRG0B;

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_A_BASE;
    pmt_cfg.config.adc16.trig_len = 2;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_VOL_ADC_CH_A;
    pmt_cfg.config.adc16.adc_ch[1] = BOARD_BLDC_VOL_ADC_CH_BUS;
    hpm_adc_set_preempt_config(&pmt_cfg);

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_B_BASE;
    pmt_cfg.config.adc16.trig_len = 1;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_VOL_ADC_CH_B;
    hpm_adc_set_preempt_config(&pmt_cfg);

    pmt_cfg.adc_base.adc16 = BOARD_BLDC_VOL_ADC_C_BASE;
    pmt_cfg.config.adc16.trig_len = 1;
    pmt_cfg.config.adc16.adc_ch[0] = BOARD_BLDC_VOL_ADC_CH_C;
    hpm_adc_set_preempt_config(&pmt_cfg);

    adc16_set_pmt_queue_enable(BOARD_BLDC_VOL_ADC_A_BASE, ADC16_CONFIG_TRG0B, true);
    adc16_set_pmt_queue_enable(BOARD_BLDC_VOL_ADC_B_BASE, ADC16_CONFIG_TRG0B, true);
    adc16_set_pmt_queue_enable(BOARD_BLDC_VOL_ADC_C_BASE, ADC16_CONFIG_TRG0B, true);

    /* Set DMA start address for preemption mode */
    hpm_adc_init_pmt_dma(&adc0, core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)adc0_dma_buf));
    hpm_adc_init_pmt_dma(&adc1, core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)adc1_dma_buf));
    hpm_adc_init_pmt_dma(&adc2, core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)adc2_dma_buf));

    /* NTC序列采集 */
    // adc16_set_nonblocking_read(BOARD_NTC1_ADC_BASE);
    // adc16_enable_busywait(BOARD_NTC1_ADC_BASE);
}

uint16_t adc_oneshot_read(ADC16_Type *ptr, uint8_t ch)
{
    uint16_t result;
    if (adc16_get_oneshot_result(ptr, ch, &result) == status_success)
    {
        if (adc16_is_nonblocking_mode(ptr))
        {
            adc16_get_oneshot_result(ptr, ch, &result);
        }
    }
    return result;
}

uint16_t adc_get_ntc1_raw()
{
    return adc_oneshot_read(BOARD_NTC1_ADC_BASE, BOARD_NTC1_ADC_CH);
}

uint16_t adc_get_ntc2_raw()
{
    return adc_oneshot_read(BOARD_NTC2_ADC_BASE, BOARD_NTC2_ADC_CH);
}

void adc_set_callback(adc_callback_t cb)
{
    __isr_callback = cb;
}

void adc_get_trigger0a_raw(uint16_t data[3])
{
    data[0] = adc0_dma_buf[0].value;
    data[1] = adc2_dma_buf[0].value;
    data[2] = adc1_dma_buf[0].value;
}

void adc_get_trigger0b_raw(uint16_t data[4])
{
    data[0] = adc1_dma_buf[4].value;
    data[1] = adc0_dma_buf[4].value;
    data[2] = adc2_dma_buf[4].value;
    data[3] = adc1_dma_buf[5].value;
}

/**
 * @brief 中断服务函数
 */
static SDK_DECLARE_EXT_ISR_M(IRQn_ADC0, __isr_adc0_fun);
static void __isr_adc0_fun(void)
{
    uint32_t status = hpm_adc_get_status_flags(&adc0);

    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0)
    {
        hpm_adc_clear_status_flags(&adc0, BOARD_BLDC_ADC_TRIG_FLAG);
        if (__isr_callback)
            __isr_callback(HPM_ADC0, BOARD_BLDC_ADC_TRIG_FLAG);
    }
}

static SDK_DECLARE_EXT_ISR_M(IRQn_ADC1, __isr_adc1_fun);
static void __isr_adc1_fun(void)
{
    uint32_t status = hpm_adc_get_status_flags(&adc1);

    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0)
    {
        hpm_adc_clear_status_flags(&adc1, BOARD_BLDC_ADC_TRIG_FLAG);
        if (__isr_callback)
            __isr_callback(HPM_ADC1, BOARD_BLDC_ADC_TRIG_FLAG);
    }
}

static SDK_DECLARE_EXT_ISR_M(IRQn_ADC2, __isr_adc2_fun);
static void __isr_adc2_fun(void)
{
    uint32_t status = hpm_adc_get_status_flags(&adc2);

    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0)
    {
        hpm_adc_clear_status_flags(&adc2, BOARD_BLDC_ADC_TRIG_FLAG);
        if (__isr_callback)
            __isr_callback(HPM_ADC2, BOARD_BLDC_ADC_TRIG_FLAG);
    }
}
