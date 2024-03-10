#include "drv832x.h"
#include "board.h"
#include "hpm_common.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_trgm_drv.h"
#include "multicore_common.h"
#include "stdbool.h"
#include <stdint.h>

static spi_control_config_t control_config = {};

#define DRV_DBG(fmt, ...) printf("[DRV] " fmt, ##__VA_ARGS__)

static void spi_transfer_mode_print(spi_control_config_t *config)
{
    uint8_t trans_mode = config->common_config.trans_mode;

    char trans_mode_table[][50] = {"write-read-together", "write-only",       "read-only", "write-read",  "read-write",
                                   "write-dummy-read",    "read-dummy-write", "no-data",   "dummy-write", "dummy-read"};

    DRV_DBG("SPI-Master transfer mode:%s\n", trans_mode_table[trans_mode]);
}

void drv832x_init_pins()
{
    HPM_IOC->PAD[IOC_PAD_PC18].FUNC_CTL = IOC_PC18_FUNC_CTL_SPI3_CSN;
    HPM_IOC->PAD[IOC_PAD_PC19].FUNC_CTL = IOC_PC19_FUNC_CTL_SPI3_MISO;
    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PC21_FUNC_CTL_SPI3_MOSI;

    HPM_IOC->PAD[BOARD_DRV8323_PIN_PWM_EN].FUNC_CTL = IOC_PZ00_FUNC_CTL_GPIO_Z_00;
    HPM_BIOC->PAD[BOARD_DRV8323_PIN_PWM_EN].FUNC_CTL = IOC_PZ00_FUNC_CTL_SOC_GPIO_Z_00;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 0, gpiom_soc_gpio0);
    gpiom_disable_pin_visibility(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 0, gpiom_soc_gpio1);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOZ, 0);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ, 0, 0);

    HPM_IOC->PAD[BOARD_DRV8323_PIN_CAL].FUNC_CTL = IOC_PZ02_FUNC_CTL_GPIO_Z_02;
    HPM_BIOC->PAD[BOARD_DRV8323_PIN_CAL].FUNC_CTL = IOC_PZ02_FUNC_CTL_SOC_GPIO_Z_02;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 0, gpiom_soc_gpio0);
    gpiom_disable_pin_visibility(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 0, gpiom_soc_gpio1);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOZ, 2);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ, 2, 0);

    HPM_IOC->PAD[BOARD_DRV8323_PIN_nFAULT].FUNC_CTL = IOC_PB26_FUNC_CTL_TRGM0_P_06;
    trgm_disable_io_output(HPM_TRGM0, 1 << 6);
}

void drv832x_enable(bool en)
{
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ, 0, en);
}

void drv832x_calibration(bool en)
{
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ, 2, en);
}

void drv832x_init_spi()
{
    spi_timing_config_t timing_config = {};
    spi_format_config_t format_config = {};

    uint32_t spi_clcok = board_init_spi_clock(BOARD_DRV8323_SPI_BASE);

    /* set SPI sclk frequency for master */
    timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
    timing_config.master_config.cs2sclk = spi_cs2sclk_half_sclk_2;
    timing_config.master_config.csht = spi_csht_half_sclk_12;
    timing_config.master_config.sclk_freq_in_hz = BOARD_DRV8323_SPI_SCLK_FREQ; // 10M
    spi_master_timing_init(BOARD_DRV8323_SPI_BASE, &timing_config);
    DRV_DBG("DRV832x SPI timing is configured.\n");
    DRV_DBG("DRV832x SPI source clock frequency: %dHz\n", timing_config.master_config.clk_src_freq_in_hz);
    DRV_DBG("DRV832x SPI sclk frequency: %dHz\n", timing_config.master_config.sclk_freq_in_hz);

    /* set SPI format config for master */
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = BOARD_DRV8323_SPI_DATA_LEN_IN_BITS;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_low_idle;
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    format_config.common_config.lsb = false;
    spi_format_init(BOARD_DRV8323_SPI_BASE, &format_config);
    DRV_DBG("DRV832x SPI format is configured.\n");

    /* set SPI control config for master */
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
    control_config.master_config.addr_enable = false; /* address phase control for master */
    control_config.common_config.trans_mode = spi_trans_write_read_together;
    spi_transfer_mode_print(&control_config);
}

int drv832x_write_reg(uint8_t addr, uint16_t data)
{
    if (addr >= DRV_ADDR_MAX)
        return status_fail;

    uint8_t tx[2] = {};
    uint8_t rx[2] = {};
    tx[0] = 0x80 | (addr << 3) | ((data >> 8) & 0x7);
    tx[1] = data & 0xff;
    return spi_transfer(BOARD_DRV8323_SPI_BASE, &control_config, NULL, NULL, tx, 2, rx, 2);
}

hpm_stat_t drv832x_read_reg(uint8_t addr, uint16_t *data)
{
    if (data == NULL || addr >= DRV_ADDR_MAX)
        return status_fail;

    uint8_t tx[2] = {};
    uint8_t rx[2] = {};

    tx[0] = 0x80 | addr << 3;
    tx[1] = 0;
    hpm_stat_t stat = spi_transfer(BOARD_DRV8323_SPI_BASE, &control_config, NULL, NULL, tx, 2, rx, 2);

    if (stat == status_success)
        *data = (rx[0] & 0x07) << 8 | rx[1];
    return stat;
}

void drv832x_dump_regs()
{
    uint16_t data;
    hpm_stat_t stat;
    DRV_DBG("dump DRV832x Regs:\n");

#define XX(ADDR)                                                                                                       \
    if ((stat = drv832x_read_reg(ADDR, &data)) == status_success)                                                      \
    {                                                                                                                  \
        DRV_DBG("[%d]%-16s: %#03x\n", ADDR, (#ADDR) + 9, data);                                                        \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        DRV_DBG("drv832x_read_reg error %d\n", stat);                                                                  \
    }

    XX(DRV_ADDR_FALULT_STA)
    XX(DRV_ADDR_VGS_STA)
    XX(DRV_ADDR_DRIVER_CTL)
    XX(DRV_ADDR_GATE_DRIVER_HS)
    XX(DRV_ADDR_GATE_DRIVER_LS)
    XX(DRV_ADDR_OCP_CTL)
#if DRV8323
    XX(DRV_ADDR_CSA_CTL)
#endif

#undef XX
}
