#pragma once

#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "stdbool.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接trgm信号
 *
 * @param input 输入源 HPM_TRGM0_INPUT_SRC_xxxxx
 * @param output 输出 HPM_TRGM0_OUTPUT_SRC_xxxxx
 * @param type 输出类型
 * @param invert 输出反转
 */
static inline void trgm_connect(TRGM_Type *ptr, uint8_t input, uint8_t output, trgm_output_type_t type, bool invert)
{
    trgm_output_t trgm_output_cfg = {
        .input = input,
        .invert = invert,
        .type = type,
    };

    trgm_output_config(ptr, output, &trgm_output_cfg);
}

#ifdef __cplusplus
}
#endif
