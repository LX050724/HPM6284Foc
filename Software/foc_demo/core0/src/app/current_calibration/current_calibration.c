#include "current_calibration.h"
#include "hardware/adc_init.h"
#include "project_config.h"
#include <stdbool.h>
#include <stdint.h>

#define CURRCAL_DEBUG(fmt, ...) DLOG("CC", fmt, ##__VA_ARGS__)

static volatile int calibration_status;
static int calibration[3];
static uint32_t calibration_count;
static MotorClass_t *gpMotor;

static void __adc_cb(ADC16_Type *adc, uint32_t flag)
{
    uint16_t adc_raw[3] = {};
    if (calibration_status != 1)
        return;

    if (calibration_count < ADC_CALIBRATION_TIMES)
    {
        adc_get_trigger0a_raw(adc_raw);
        calibration[0] += adc_raw[0];
        calibration[1] += adc_raw[1];
        calibration[2] += adc_raw[2];
        calibration_count += 1;
    }
    else
    {
        calibration[0] /= ADC_CALIBRATION_TIMES;
        calibration[1] /= ADC_CALIBRATION_TIMES;
        calibration[2] /= ADC_CALIBRATION_TIMES;
        current_set_calibration(&gpMotor->current_cal, calibration);
        calibration_status = 2;
    }
}

int current_calibration(MotorClass_t *motor)
{
    motor->enable_pwm(motor, false);
    CPU_Delay(500);

    gpMotor = motor;
    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;
    calibration_count = 0;
    calibration_status = 1;
    adc_set_callback(__adc_cb);
    adc_enable_irq(1);
    adc_enable_it();

    while (calibration_status != 2)
    {
    }

    adc_disable_it();
    adc_disable_irq();
    adc_set_callback(NULL);
    gpMotor = NULL;

    CURRCAL_DEBUG("current calibration done");
    CURRCAL_DEBUG("  A offset %dmV", (int)(adc_voltage(calibration[0]) * 1000));
    CURRCAL_DEBUG("  B offset %dmV", (int)(adc_voltage(calibration[1]) * 1000));
    CURRCAL_DEBUG("  C offset %dmV", (int)(adc_voltage(calibration[2]) * 1000));
    return 0;
}