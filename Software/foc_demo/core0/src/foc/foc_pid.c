#include "foc/foc_pid.h"

void foc_pid_init(foc_pid_contrl_t *pid)
{
    pid->err[0] = 0;
    pid->err[1] = 0;
    pid->integral = 0;
}

float foc_pi_controller(foc_pid_contrl_t *pid, float cur, float exp)
{
    float err = exp - cur;
    pid->integral = foc_pid_limit(pid->integral + err, pid->integral_limit);
    float output = pid->kp * err + pid->ki * pid->integral;
    return foc_pid_limit(output, pid->output_limit);
}

float foc_pid_controller(foc_pid_contrl_t *pid, float cur, float d, float exp)
{
    float err = exp - cur;
    pid->integral = foc_pid_limit(pid->integral + err, pid->integral_limit);
    float output = pid->kp * err + pid->ki * pid->integral + pid->kd * d;
    return foc_pid_limit(output, pid->output_limit);
}

float foc_pid_increase_controller(foc_pid_contrl_t *pid, float cur, float exp)
{
    float err = exp - cur;
    float output = pid->kp * (err - pid->err[0]) + pid->ki * err + pid->kd * (err - 2.0f * pid->err[0] + pid->err[1]);
    pid->err[1] = pid->err[0];
    pid->err[0] = err;
    return output;
}