#pragma once

#include "app/MotorClass.h"
#ifdef __cplusplus
extern "C" {
#endif

void force_drive(MotorClass_t *motor, float ang, float power);
int electrical_angle_calibration(MotorClass_t *motor);

#ifdef __cplusplus
}
#endif