#pragma once
#include "board.h"
#include "foc/foc_core.h"
#include "project_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t pole_pairs;
    uint16_t ang_offset;
    uint16_t a_max;
    uint16_t a_min;
    uint16_t b_max;
    uint16_t b_min;
} encoder_t;

typedef void (*ecnoder_callback_t)(uint32_t flag);

#if ENCODER_TYPE == ENCODER_MT6701
#include "mt6701_spi.h"

static inline uint16_t encoder_get_rawAngle()
{
    return mt6701_read_angle();
}

#endif

#if GET_ENCODER_INTERFACE(ENCODER_TYPE) == ENCODER_LINEAR_HALL
float encoder_get_eleAngle(const encoder_t *self)
{
    return 0.0f;
}

void encoder_get_eleAngle_sincos(const encoder_t *self, foc_sin_cos_t *sincos)
{
}
#else
static inline float encoder_get_eleAngle(const encoder_t *self, uint16_t raw_ang)
{
    return ((int32_t)(raw_ang - self->ang_offset) * self->pole_pairs) * (2.0f * F_PI / 65536.0f);;
}

static inline void encoder_get_eleAngle_sincos(const encoder_t *self, uint16_t raw_ang, foc_sin_cos_t *sincos)
{
    foc_sin_cos(encoder_get_eleAngle(self, raw_ang), sincos);
}

#endif

#ifdef __cplusplus
}
#endif