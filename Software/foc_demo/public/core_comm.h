#pragma once
#include "hpm_l1c_drv.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUF_NUM (2048 * 2 / sizeof(just_float_data))

static inline void DCache_flush(void *addr, size_t size)
{
    uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(addr);
    uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(addr + size);
    l1c_dc_flush(aligned_start, aligned_end - aligned_start);
}

static inline void DCache_wrtieback(void *addr, size_t size)
{
    uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(addr);
    uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(addr + size);
    l1c_dc_writeback(aligned_start, aligned_end - aligned_start);
}

static inline void DCache_invalidate(void *addr, size_t size)
{
    uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(addr);
    uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(addr + size);
    l1c_dc_invalidate(aligned_start, aligned_end - aligned_start);
}


typedef enum {
    INVALID_MSG = -1,
    CORE1_CONTROL_MSG,
    CORE0_VOFA_UPLOAD_BUF1,
    CORE0_VOFA_UPLOAD_BUF2,
} MsgID;

typedef struct
{
    float data[15];
    uint8_t tail[4];
} just_float_data;

typedef struct{
    uint32_t set_offset;
    float exp_iq;
    float exp_id;
    float id_p;
    float id_i;
    float id_il;
    float iq_p;
    float iq_i;
    float iq_il;
    float speed_p;
    float speed_i;
    float exp_speed;
    float ang_p;
    float ang_i;
    float exp_ang;
    float speed_filter;
} SetData_t;

typedef struct {
    just_float_data vofa_buf[BUF_NUM];
    SetData_t set_data;
} core_comm_ctl_t;

#ifdef __cplusplus
}
#endif