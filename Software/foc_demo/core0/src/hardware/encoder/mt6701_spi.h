#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mt6701_spi_init();
uint16_t mt6701_read_angle();

#ifdef __cplusplus
}
#endif