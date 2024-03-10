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

#define DRV_REG_FAULT_STA_FAULT DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 10)
#define DRV_REG_FAULT_STA_VDS_OCP DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 9)
#define DRV_REG_FAULT_STA_GDF DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 8)
#define DRV_REG_FAULT_STA_UVLO DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 7)
#define DRV_REG_FAULT_STA_OTSD DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 6)
#define DRV_REG_FAULT_STA_VDS_HA DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 5)
#define DRV_REG_FAULT_STA_VDS_LA DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 4)
#define DRV_REG_FAULT_STA_VDS_HB DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 3)
#define DRV_REG_FAULT_STA_VDS_LB DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 2)
#define DRV_REG_FAULT_STA_VDS_HC DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 1)
#define DRV_REG_FAULT_STA_VDS_LC DRV_REG_DEF(DRV_ADDR_FALULT_STA, 0x1, 0)

#define DRV_REG_VGS_STA_SA_OC DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 10)
#define DRV_REG_VGS_STA_SB_OC DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 9)
#define DRV_REG_VGS_STA_SC_OC DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 8)
#define DRV_REG_VGS_STA_OTW DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 7)
#define DRV_REG_VGS_STA_CPUV DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 6)
#define DRV_REG_VGS_STA_VGS_HA DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 5)
#define DRV_REG_VGS_STA_VGS_LA DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 4)
#define DRV_REG_VGS_STA_VGS_HB DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 3)
#define DRV_REG_VGS_STA_VGS_LB DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 2)
#define DRV_REG_VGS_STA_VGS_HC DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 1)
#define DRV_REG_VGS_STA_VGS_LC DRV_REG_DEF(DRV_ADDR_VGS_STA, 0x1, 0)

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
#define DRV_REG_SHIF(REG) ((REG)&0xff)

#define DRV_DECODE_REG(REG, VAL) (((VAL) >> DRV_REG_SHIF(REG)) & DRV_REG_MASK(REG))
#define DRV_ENCODE_REG(REG, VAL) (((VAL)&DRV_REG_MASK(REG)) << DRV_REG_SHIF(REG))

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

/**
 * @brief 打印寄存器
 * 
 */
void drv832x_dump_regs();

#ifdef __cplusplus
}
#endif