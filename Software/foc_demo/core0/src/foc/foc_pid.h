#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float err[2];
    float integral;
    float integral_limit;
    float output_limit;
} foc_pid_contrl_t;

void foc_pid_init(foc_pid_contrl_t *pid);

float foc_pi_controller(foc_pid_contrl_t *pid, float cur, float exp);
float foc_pid_controller(foc_pid_contrl_t *pid, float cur, float d, float exp);
float foc_pid_increase_controller(foc_pid_contrl_t *pid, float cur, float exp);


static inline float foc_pid_limit(float in, float limit)
{
    if (in > limit)
        return limit;
    if (in < -limit)
        return -limit;
    return in;
}

static inline float foc_pid_limit2(float in, float max, float min)
{
    if (in > max)
        return max;
    if (in < min)
        return min;
    return in;
}

static inline int foc_pid_diff(int a, int b, int rand)
{
    int diff = a - b;
    if (diff > rand / 2)
        diff -= rand;
    if (diff < -rand / 2)
        diff += rand;
    return diff;
}

#ifdef __cplusplus
}
#endif