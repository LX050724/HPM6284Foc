#pragma once

#include <project_config.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

void mt6701_spi_init();
uint16_t mt6701_read_angle();

#if ENCODER_TYPE == ENCODER_MT6701

static inline uint16_t encoder_get_rawAngle()
{
    return mt6701_read_angle();
}

#endif

#ifdef __cplusplus
}
#endif