#include "board.h"
#include "hpm_soc.h"
#include "hpm_spi_drv.h"

spi_control_config_t control_config = {};
uint16_t mt_ang;

static void mt6701_init_pins()
{
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_SPI0_CSN;
    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_SPI0_MISO;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_SPI0_MOSI;
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_SPI0_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;

    HPM_IOC->PAD[IOC_PAD_PA28].PAD_CTL =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(2);
    HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL =
        IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(2);
}

void mt6701_spi_init()
{
    spi_timing_config_t timing_config = {};
    spi_format_config_t format_config = {};
    uint32_t spi_clcok = board_init_spi_clock(HPM_SPI0);
    mt6701_init_pins();

    /* set SPI sclk frequency for master */
    timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
    timing_config.master_config.cs2sclk = spi_cs2sclk_half_sclk_3;
    timing_config.master_config.csht = spi_csht_half_sclk_1;
    timing_config.master_config.sclk_freq_in_hz = 12000000UL; // 10M
    spi_master_timing_init(HPM_SPI0, &timing_config);

    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 16;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(HPM_SPI0, &format_config);

    /* set SPI control config for master */
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
    control_config.master_config.addr_enable = false; /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_read_only;
}

uint16_t mt6701_read_angle()
{
    uint16_t data;
    spi_transfer(HPM_SPI0, &control_config, NULL, NULL, NULL, 0, (uint8_t *)&data, 1);
    return data & ~0x0003;
}