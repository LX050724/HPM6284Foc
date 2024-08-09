#pragma once

#include "hpm_common.h"
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DLOG(NAME, fmt, ...) printf("[" NAME "] " fmt "\n", ##__VA_ARGS__)

/* 编码器接口类型 */
#define ENCODER_INTERFACE_IIC 2
#define ENCODER_INTERFACE_SPI 3
#define ENCODER_INTERFACE_QEI 4
#define ENCODER_INTERFACE_LINEAR_HALL 5
#define ENCODER_INTERFACE_ANALOG 6

/* 编码器类型 */
#define ENCODER_MT6701 (ENCODER_INTERFACE_SPI << 8 | 1)
#define ENCODER_UVW (ENCODER_INTERFACE_QEI << 8 | 2)
#define ENCODER_ABZ (ENCODER_INTERFACE_QEI << 8 | 3)
#define ENCODER_LINEAR_HALL (ENCODER_INTERFACE_LINEAR_HALL << 8 | 4)

#define ENCODER_TYPE ENCODER_MT6701 // 编码器类型
#define ENCODER_MAX 0xffffff        // 编码器最大值
#define ENCODER_INVALID ((uint32_t) - 1)

/* 时间参数 */
#define PWM_FREQUENCY 50000                     // PWM频率(Hz) 根据散热条件 Max: 100KHz@24V, 50Khz@48V
#define SPEED_PID_FREQUENCY 5000                // 速度、位置环频率(Hz)
#define ELECTRICAL_ANGLE_CALIBRATION_POWER 0.4f // 电角度校准油门
#define ELECTRICAL_ANGLE_CALIBRATION_DELAY 500  // 电角度校准延迟(ms)

/* ADC参数 */
#define SAMPLING_RESISTOR 0.005f                      // 采样电阻 Ω
#define CURRENT_AMP 20                                // 电流运放放大倍数 5/10/20/40
#define CURRENT_COE (SAMPLING_RESISTOR * CURRENT_AMP) // 电流系数 V/A
#define VOLTAGE_AMP 20.0f                             // 母线电压放大倍数
#define ADC_IGNORE_BIT 5                              // ADC低位舍弃
#define ADC_ENABLE_FILTER 1                           // 启用2位滑动平均滤波
#define ADC_CALIBRATION_TIMES 1024                    // ADC校准采样次数

/* 启用HRPWM */
#define HRPWM_ENABLE 0 // HRPWM故障保护功能有问题，暂不开启

/* 速度滤波方式 */
#define SPEED_FILTER_PLL 0  // 使用电角度计算PLL
#define SPEED_FILTER_PLL2 1 // 使用机械角度计算简化PLL
#define SPEED_FILTER_IIR 2  // 使用IIR滤波器，需要根据PWM频率调整滤波器系数，运算量较大

#define SPEED_FILTER_MODE SPEED_FILTER_IIR

/**
 * @brief 电压限幅参数
 * @note 限幅矢量模不能超过 2/√3 * 0.96
 *       其中0.96为PWM最大占空比
 */
#define UQ_LIMIT 0.8f // Q轴电压限幅
#define UD_LIMIT 0.2f // D轴电压限幅

/* 不可修改 */
#define AHB_CLOCK 200000000LLU // AHB时钟频率(Hz)

#if HRPWM_ENABLE
#define PWM_RELOAD (((((AHB_CLOCK) << 8) / (PWM_FREQUENCY)) >> 8) & 0xffffff) // PWM重载值
#define HRPWM_RELOAD ((((AHB_CLOCK) << 8) / (PWM_FREQUENCY)) & 0xff)          // HRPWM重载值
#define PWM_MAX ((AHB_CLOCK / PWM_FREQUENCY) << 8)                            // PWM最大值
#else
#define PWM_RELOAD ((AHB_CLOCK) / (PWM_FREQUENCY)) // PWM重载值
#define HRPWM_RELOAD 0
#define PWM_MAX PWM_RELOAD // PWM最大值
#endif

#if ADC_CALIBRATION_TIMES > 65536
#error "ADC_CALIBRATION_TIMES > 65536"
#endif

#if ENCODER_MAX > 0xffffff
#error "ENCODER_MAX > 0xffffff"
#endif

#define GET_ENCODER_INTERFACE(x) (((x) >> 8) & 0xff)
#define DMA_ATTR ATTR_PLACE_AT_NONCACHEABLE ATTR_ALIGN(64)

#define CPU_Delay(ms) clock_cpu_delay_ms(ms)

#ifdef __cplusplus
}
#endif