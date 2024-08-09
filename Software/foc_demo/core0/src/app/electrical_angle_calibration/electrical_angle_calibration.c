#include "app/MotorClass.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "hardware/encoder/encoder.h"
#include "hpm_clock_drv.h"
#include "project_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define EACAL_DEBUG(fmt, ...) DLOG("EC", fmt, ##__VA_ARGS__)

void force_drive(MotorClass_t *motor, float ang, float power)
{
    foc_sin_cos_t ang_sincos = {};
    foc_qd_current_t qd_current = {};
    foc_alpha_beta_volt_t alpha_beta_volt = {};
    foc_pwm_t pwm = {};

    qd_current.id = power;
    foc_sin_cos(ang, &ang_sincos);
    foc_inv_park(&qd_current, &ang_sincos, &alpha_beta_volt);
    foc_svpwm(&alpha_beta_volt, &pwm);
    motor->set_pwm_cb(motor, &pwm);
}

float_t LeastSquareLinearFit(const int32_t y[], const int num, float_t *k, float_t *b)
{
    float_t sum_x2 = 0;
    float_t sum_y = 0;
    float_t sum_x = 0;
    float_t sum_xy = 0;

    for (int i = 0; i < num; i++)
    {
        float_t x = F_PI * i / 6;
        sum_x2 += x * x;
        sum_y += y[i];
        sum_x += x;
        sum_xy += x * y[i];
    }
    float_t ave_x = sum_x / num, ave_y = sum_y / num;

    *k = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);
    *b = ave_y - (*k) * ave_x;

    float_t sum_cov = 0;
    float_t sum_dx2 = 0, sum_dy2 = 0;
    for (int i = 0; i < num; i++)
    {
        float_t dx = F_PI * i / 6 - ave_x;
        float_t dy = y[i] - ave_y;
        sum_cov += dx * dy;
        sum_dx2 += dx * dx;
        sum_dy2 += dy * dy;
    }

    return powf(sum_cov / (sqrtf(sum_dx2) * sqrtf(sum_dy2)), 2);
}

static int32_t get_ang(MotorClass_t *motor)
{
    uint32_t t;
    int retry = 0;
    do
    {
        t = motor->get_raw_angle_cb(motor);
    } while (t != ENCODER_INVALID && retry++ < 5);
    return (int32_t)t;
}
int electrical_angle_calibration(MotorClass_t *motor)
{
    int32_t ang_table_f[12];
    int32_t ang_table_b[12];
    float_t k, b, r2;
    int16_t pole_pairs;
    uint32_t offset;

    EACAL_DEBUG("electrical angle calibration start ...");
    motor->enable_pwm(motor, true);

    force_drive(motor, 0, ELECTRICAL_ANGLE_CALIBRATION_POWER);
    CPU_Delay(ELECTRICAL_ANGLE_CALIBRATION_DELAY * 2);

    for (int i = 0; i < 12; i++)
    {
        force_drive(motor, F_PI * i / 6, ELECTRICAL_ANGLE_CALIBRATION_POWER);
        CPU_Delay(ELECTRICAL_ANGLE_CALIBRATION_DELAY);
        ang_table_f[i] = get_ang(motor);
        if (ang_table_f[i] == ENCODER_INVALID)
            goto _error;
    }

    /* 处理溢出 */
    for (int i = 0; i < 11; i++)
    {
        int32_t diff = ang_table_f[i] - ang_table_f[i + 1];
        if (diff > ENCODER_MAX / 2)
            ang_table_f[i + 1] += ENCODER_MAX;
        else if (diff < -ENCODER_MAX / 2)
            ang_table_f[i + 1] -= ENCODER_MAX;
    }
    for (int i = 0; i < 12; i++)
    {
        EACAL_DEBUG("1 %2d %d", i, ang_table_f[i]);
    }

    force_drive(motor, 0, ELECTRICAL_ANGLE_CALIBRATION_POWER);
    CPU_Delay(ELECTRICAL_ANGLE_CALIBRATION_DELAY);

    for (int i = 11; i >= 0; i--)
    {
        force_drive(motor, F_PI * i / 6, ELECTRICAL_ANGLE_CALIBRATION_POWER);
        CPU_Delay(ELECTRICAL_ANGLE_CALIBRATION_DELAY);
        ang_table_b[i] = get_ang(motor);
        if (ang_table_b[i] == ENCODER_INVALID)
            goto _error;
    }

    /* 处理溢出 */
    for (int i = 0; i < 11; i++)
    {
        int32_t diff = ang_table_b[i] - ang_table_b[i + 1];
        if (diff > ENCODER_MAX / 2)
            ang_table_b[i + 1] += ENCODER_MAX;
        else if (diff < -ENCODER_MAX / 2)
            ang_table_b[i + 1] -= ENCODER_MAX;
    }

    for (int i = 0; i < 12; i++)
    {
        ang_table_f[i] = (ang_table_f[i] + ang_table_b[i]) / 2;
        EACAL_DEBUG("1 %2d %d %d", i, ang_table_b[i], ang_table_f[i]);
    }

    motor->enable_pwm(motor, false);

    /* 线性回归 */

    r2 = LeastSquareLinearFit(ang_table_f, 12, &k, &b);
    EACAL_DEBUG("LeastSquareLinearFit: y=%dx%+d, r2=%de-3", (int)k, (int)b, (int)(r2 * 1000));

    if (r2 < 0.95f)
    {
        EACAL_DEBUG("calibration error: The correlation is insufficient, r2=%de-3", (int)(r2 * 1000));
        return -1;
    }
    pole_pairs = lroundf(1.0f / (fabsf(k) * 2 * F_PI / ENCODER_MAX));
    offset = lroundf(b);
    if (k < 0)
    {
        // offset += ENCODER_MAX / 2 / pole_pairs;
        pole_pairs = -pole_pairs;
    }
    EACAL_DEBUG("calibration success: direction = %u, offset = %d, pole_pairs = %d", k > 0, offset, pole_pairs);

    motor->encoder.pole_pairs = pole_pairs;
    motor->encoder.ang_offset = offset;

    return 0;

_error:
    motor->enable_pwm(motor, false);
    EACAL_DEBUG("other error");
    return -1;
}