/**
 * @file drv832x.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-03-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "hpm_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV8323 1

#define DRV_ADDR_FALULT_STA 0
#define DRV_ADDR_VGS_STA 1
#define DRV_ADDR_DRIVER_CTL 2
#define DRV_ADDR_GATE_DRIVER_HS 3
#define DRV_ADDR_GATE_DRIVER_LS 4
#define DRV_ADDR_OCP_CTL 5

#if DRV8323
#define DRV_ADDR_CSA_CTL 6 // DRV8323 Only
#define DRV_ADDR_MAX 8
#else
#define DRV_ADDR_MAX 7
#endif

#define DRV_REG_DEF(ADDR, MASK, OFFSET) ((ADDR) << 16 | (MASK) << 8 | (OFFSET))

#define DRV_REG_DRIVER_CTL_DIS_CPUV DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 9)
#define DRV_REG_DRIVER_CTL_DIS_GDF DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 8)
#define DRV_REG_DRIVER_CTL_OTW_REP DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 7)
#define DRV_REG_DRIVER_CTL_PWM_MODE DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 3, 5)
#define DRV_REG_DRIVER_CTL_1PWM_COM DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 4)
#define DRV_REG_DRIVER_CTL_1PWM_DIR DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 3)
#define DRV_REG_DRIVER_CTL_COAST DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 2)
#define DRV_REG_DRIVER_CTL_BRAKE DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 1)
#define DRV_REG_DRIVER_CTL_CLR_FLT DRV_REG_DEF(DRV_ADDR_DRIVER_CTL, 0x1, 0)

#define DRV_REG_DRIVER_GATE_DRIVER_HS_LOCK DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_HS, 0x7, 8)
#define DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERP_HS DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_HS, 0xf, 4)
#define DRV_REG_DRIVER_GATE_DRIVER_HS_IDRIVERN_HS DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_HS, 0xf, 0)

#define DRV_REG_DRIVER_GATE_DRIVER_LS_CBC DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_LS, 0x1, 10)
#define DRV_REG_DRIVER_GATE_DRIVER_LS_TDRIVE DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_LS, 0x3, 8)
#define DRV_REG_DRIVER_GATE_DRIVER_LS_IDRIVERP_LS DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_LS, 0x3, 4)
#define DRV_REG_DRIVER_GATE_DRIVER_LS_IDRIVERN_LS DRV_REG_DEF(DRV_ADDR_GATE_DRIVER_LS, 0xf, 0)

#define DRV_REG_OCP_CTL_TRETRY DRV_REG_DEF(DRV_ADDR_OCP_CTL, 0x1, 10)
#define DRV_REG_OCP_CTL_DEAD_TIME DRV_REG_DEF(DRV_ADDR_OCP_CTL, 0x3, 8)
#define DRV_REG_OCP_CTL_OCP_MODE DRV_REG_DEF(DRV_ADDR_OCP_CTL, 0x3, 6)
#define DRV_REG_OCP_CTL_OCP_DEG DRV_REG_DEF(DRV_ADDR_OCP_CTL, 0x3, 4)
#define DRV_REG_OCP_CTL_VDS_LVL DRV_REG_DEF(DRV_ADDR_OCP_CTL, 0xf, 0)

#define DRV_REG_CSA_CTL_CSA_FET DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 10)
#define DRV_REG_CSA_CTL_VREF_DIV DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 9)
#define DRV_REG_CSA_CTL_LS_REF DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 8)
#define DRV_REG_CSA_CTL_CSA_GAIN DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x3, 6)
#define DRV_REG_CSA_CTL_DIS_SEN DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 5)
#define DRV_REG_CSA_CTL_CSA_CAL_A DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 4)
#define DRV_REG_CSA_CTL_CSA_CAL_B DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 3)
#define DRV_REG_CSA_CTL_CSA_CAL_C DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x1, 2)
#define DRV_REG_CSA_CTL_SEN_LVL DRV_REG_DEF(DRV_ADDR_CSA_CTL, 0x3, 0)

#define DRV_REG_ADDR(REG) (((REG) >> 16) & 0xff)
#define DRV_REG_MASK(REG) (((REG) >> 8) & 0xff)
#define DRV_REG_SHIF(REG) ((REG) & 0xff)

typedef enum
{
    DRV_FAULT_VDS_LC = 0x00000001,  // IndicatesVDS overcurrent fault on theC low-side MOSFET
    DRV_FAULT_VDS_HC = 0x00000002,  // IndicatesVDS overcurrent fault on theC high-side MOSFET
    DRV_FAULT_VDS_LB = 0x00000004,  // IndicatesVDS overcurrent fault on theB low-side MOSFET
    DRV_FAULT_VDS_HB = 0x00000008,  // IndicatesVDS overcurrent fault on theB high-side MOSFET
    DRV_FAULT_VDS_LA = 0x00000010,  // IndicatesVDS overcurrent fault on theA low-side MOSFET
    DRV_FAULT_VDS_HA = 0x00000020,  // IndicatesVDS overcurrent fault on theA high-side MOSFET
    DRV_FAULT_OTSD = 0x00000040,    // Indicatesovertemperature shutdown
    DRV_FAULT_UVLO = 0x00000080,    // Indicatesundervoltage lockout fault condition
    DRV_FAULT_GDF = 0x00000100,     // Indicatesgate drive fault condition
    DRV_FAULT_VDS_OCP = 0x00000200, // IndicatesVDS monitor overcurrent fault condition
    DRV_FAULT_FAULT = 0x00000400,   // Logic OR of FAULT status registers. Mirrors nFAULT pin.
    DRV_FAULT_VGS_LC = 0x00000800,  // Indicatesgate drive fault on theC low-side MOSFET
    DRV_FAULT_VGS_HC = 0x00001000,  // Indicatesgate drive fault on theC high-side MOSFET
    DRV_FAULT_VGS_LB = 0x00002000,  // Indicatesgate drive fault on theB low-side MOSFET
    DRV_FAULT_CPUV = 0x00004000,    // Indicatescharge pump undervoltage fault condition
    DRV_FAULT_VGS_HB = 0x00008000,  // Indicatesgate drive fault on theB high-side MOSFET
    DRV_FAULT_VGS_LA = 0x00010000,  // Indicatesgate drive fault on theA low-side MOSFET
    DRV_FAULT_VGS_HA = 0x00020000,  // Indicatesgate drive fault on theA high-side MOSFET
    DRV_FAULT_OTW = 0x00040000,     // Indicatesovertemperature warning
    DRV_FAULT_SC_OC = 0x00080000,   // Indicatesovercurrent on phase C sense amplifier (DRV8323xS)
    DRV_FAULT_SB_OC = 0x00100000,   // Indicatesovercurrent on phase B sense amplifier (DRV8323xS)
    DRV_FAULT_SA_OC = 0x00200000,   // Indicatesovercurrent on phase A sense amplifier (DRV8323xS)
} DRV_FAULT_STAT;

/**
 * @brief Driver Control Register (address = 0x02)
 *
 */

enum
{
    DIS_CPUV_ENABLE,  // 0b = Charge pump UVLO fault is enabled
    DIS_CPUV_DISABLE, // 1b = Charge pump UVLO fault isdisabled
};

enum
{
    DIS_GDF_ENABLE,  // 0b = Gate driv e fault is enabled
    DIS_GDF_DISABLE, // 1b = Gate drive fault isdisabled
};

enum
{
    OTW_REP_DISABLE, // 0b = OTW is not reported on nFAULT or the FAULT bit
    OTW_REP_ENABLE,  // 1b = OTW is reported on nFAULT and the FAULT bit
};

enum
{
    PWM_MODE_6x,          // 00b = 6x PWM Mode
    PWM_MODE_3x,          // 01b = 3x PWM mode
    PWM_MODE_1x,          // 10b = 1x PWM mode
    PWM_MODE_Independent, // 11b = Independent PWM mode
};

enum
{
    ONE_PWM_COM_SYNC,  // 0b = 1x PWM mode uses synchronous rectification
    ONE_PWM_COM_ASYNC, // 1b = 1x PWM mode uses asynchronous rectification (diode freewheeling)
};

/**
 * @brief Gate Drive LS Register (address = 0x04)
 *
 */

enum
{
    IDRIVEP_10mA,   // 0000b = 10 mA
    IDRIVEP_30mA,   // 0001b = 30 mA
    IDRIVEP_60mA,   // 0010b = 60 mA
    IDRIVEP_80mA,   // 0011b = 80 mA
    IDRIVEP_120mA,  // 0100b = 120 mA
    IDRIVEP_140mA,  // 0101b = 140 mA
    IDRIVEP_170mA,  // 0110b = 170 mA
    IDRIVEP_190mA,  // 0111b = 190 mA
    IDRIVEP_260mA,  // 1000b = 260 mA
    IDRIVEP_330mA,  // 1001b = 330 mA
    IDRIVEP_370mA,  // 1010b = 370 mA
    IDRIVEP_440mA,  // 1011b = 440 mA
    IDRIVEP_570mA,  // 1100b = 570 mA
    IDRIVEP_680mA,  // 1101b = 680 mA
    IDRIVEP_820mA,  // 1110b = 820 mA
    IDRIVEP_1000mA, // 1111b = 1000 mA
};

enum
{
    IDRIVEN_20mA,   // 0000b = 20 mA
    IDRIVEN_60mA,   // 0001b = 60 mA
    IDRIVEN_120mA,  // 0010b = 120 mA
    IDRIVEN_160mA,  // 0011b = 160 mA
    IDRIVEN_240mA,  // 0100b = 240 mA
    IDRIVEN_280mA,  // 0101b = 280 mA
    IDRIVEN_340mA,  // 0110b = 340 mA
    IDRIVEN_380mA,  // 0111b = 380 mA
    IDRIVEN_520mA,  // 1000b = 520 mA
    IDRIVEN_660mA,  // 1001b = 660 mA
    IDRIVEN_740mA,  // 1010b = 740 mA
    IDRIVEN_880mA,  // 1011b = 880 mA
    IDRIVEN_1140mA, // 1100b = 1140 mA
    IDRIVEN_1360mA, // 1101b = 1360 mA
    IDRIVEN_1640mA, // 1110b = 1640 mA
    IDRIVEN_2000mA, // 1111b = 2000 mA
};

enum
{
    GDLS_CBC_DISABLE,
    GDLS_CBC_ENABLE, // Cycle-by cycle operation.  In retry OCP_MODE, for both
                     // VDS_OCP and SEN_OCP, the fault is automatically cleared
                     // when a PWM input isgiven
};

enum
{
    GDLS_TDRIVE_500ns,  // 00b = 500-ns peak gate-current drive time
    GDLS_TDRIVE_1000ns, // 01b = 1000-ns peak gate-current drive time
    GDLS_TDRIVE_2000ns, // 10b = 2000-ns peak gate-current drive time
    GDLS_TDRIVE_4000ns, // 11b = 4000-ns peak gate-current driv e time
};

/**
 * @brief OCP Control Register (address = 0x05)
 *
 */

enum
{
    OCP_TRETRY_4MS,  // 0b = VDS_OCP and SEN_OCP retry time is 4 ms
    OCP_TRETRY_50us, // 1b = VDS_OCP and SEN_OCP retry time is 50 µs
};

enum
{
    OCP_DEAD_TIME_50ns,  // 00b = 50-ns dead time
    OCP_DEAD_TIME_100ns, // 01b = 100-ns dead time
    OCP_DEAD_TIME_200ns, // 10b = 200-ns dead time
    OCP_DEAD_TIME_400ns, // 11b = 400-ns dead time
};

enum
{
    OCP_MODE_LARCHED,    // 00b = Overcurrent causes a latched fault
    OCP_MODE_AUTO_RETRY, // 01b = Overcurrent causes an automatic retrying fault
    OCP_MODE_REPORT,     // 10b = Overcurrent is report only but no action istaken
    OCP_MODE_NONE,       // 11b = Overcurrent is not reported and no action istaken
};

enum
{
    OCP_DEG_2us, // 00b = Overcurrent deglitch time of 2 µs
    OCP_DEG_4us, // 01b = Overcurrent deglitch time of 4 µs
    OCP_DEG_6us, // 10b = Overcurrent deglitch time of 6 µs
    OCP_DEG_8us, // 11b = Overcurrent deglitch time of 8 µs
};

enum
{
    OCP_VDS_LVL_60mV,   // 0000b = 0.06 V
    OCP_VDS_LVL_130mV,  // 0001b = 0.13 V
    OCP_VDS_LVL_200mV,  // 0010b = 0.2 V
    OCP_VDS_LVL_260mV,  // 0011b = 0.26 V
    OCP_VDS_LVL_310mV,  // 0100b = 0.31 V
    OCP_VDS_LVL_450mV,  // 0101b = 0.45 V
    OCP_VDS_LVL_530mV,  // 0110b = 0.53 V
    OCP_VDS_LVL_600mV,  // 0111b = 0.6 V
    OCP_VDS_LVL_680mV,  // 1000b = 0.68 V
    OCP_VDS_LVL_750mV,  // 1001b = 0.75 V
    OCP_VDS_LVL_940mV,  // 1010b = 0.94 V
    OCP_VDS_LVL_1130mV, // 1011b = 1.13 V
    OCP_VDS_LVL_1300mV, // 1100b = 1.3 V
    OCP_VDS_LVL_1500mV, // 1101b = 1.5 V
    OCP_VDS_LVL_1700mV, // 1110b = 1.7 V
    OCP_VDS_LVL_1880mV, // 1111b = 1.88 V
};

/**
 * @brief CSA Control Register (DRV8323x Only) (address= 0x06)
 */

enum
{
    CSA_FET_SPX, // Current sense amplifier positiv e input is SPx
    CSA_FET_SHX, // Current sense amplifier positive input is SHx (also automatically sets the LS_REF bit to 1)
};

enum
{
    CSA_VREF_DIV_1, // Current sense amplifier reference voltage is VREF (unidirectional mode)
    CSA_VREF_DIV_2, // Current sense amplifier reference voltage is VREF div ided by 2
};

enum
{
    CSA_LS_REF_SH2SP, // VDS_OCP for the low-side MOSFET is measured across SHx to SPx
    CSA_LS_REF_SH2SN, // VDS_OCP for the low-side MOSFET is measured across SHx to SNx
};

enum
{
    CSA_GAIN_5,  // 00b = 5-V/V current sense amplifier gain
    CSA_GAIN_10, // 01b = 10-V/V current sense amplifier gain
    CSA_GAIN_20, // 10b = 20-V/V current sense amplifier gain
    CSA_GAIN_40, // 11b = 40-V/V current sense amplifier gain
};

enum
{
    DIS_SEN_ENABLE,  // 0b = Sense ov ercurrent fault is enabled
    DIS_SEN_DISABLE, // 1b = Sense overcurrent fault isdisabled
};

enum
{
    CSA_CAL_STOP,  // 0b = Normal current sense amplifier A operation
    CSA_CAL_START, // 1b = Short inputs to current sense amplifier A for offset calibration
};

enum
{
    CSA_SEN_LVL_250mV,  // 00b = Sense OCP 0.25 V
    CSA_SEN_LVL_500mV,  // 01b = Sense OCP 0.5 V
    CSA_SEN_LVL_750mV,  // 10b = Sense OCP 0.75 V
    CSA_SEN_LVL_1000mV, // 11b = Sense OCP 1 V
};

/**
 * }
 *
 */

/**
 * @brief 初始化引脚
 *
 */
void drv832x_init_pins();

/**
 * @brief 初始化SPI接口
 *
 */
void drv832x_init_spi();

/**
 * @brief 配置EN引脚
 *
 */
void drv832x_enable(bool);

/**
 * @brief 配置CAL引脚
 *
 */
void drv832x_calibration(bool);

/**
 * @brief
 *
 * @param addr 寄存器地址
 * @param data 写入数据
 * @return int
 */
int drv832x_write_reg(uint8_t addr, uint16_t data);

/**
 * @brief
 *
 * @param addr 寄存器地址
 * @param data 返回数据指针
 * @return hpm_stat_t
 */
hpm_stat_t drv832x_read_reg(uint8_t addr, uint16_t *data);

int drv832x_get(uint32_t reg);
int drv832x_set(uint32_t reg, uint8_t val);

int drv832x_unlock();
int drv832x_lock();
/**
 * @brief 打印寄存器
 *
 */
void drv832x_dump_regs();

#ifdef __cplusplus
}
#endif
