#include "pid.h"

#define PID_EINT_MAX 0.2f

/* PID state */
static float pid_eint_steer = 0;
static float pid_eprev_steer = 0;
static float pid_eint_throttle = 0;
static float pid_eprev_throttle = 0;

static float const pid_kp_steer = 0.85f;
static float const pid_ki_steer = 0.35f;
static float const pid_kd_steer = 0.06f;

static float const pid_kp_throttle = 0.8f;
static float const pid_ki_throttle = 0.0f;
static float const pid_kd_throttle = 0.4f;

/**
 * @brief Slightly improved PID controller.
 * @param kp Proportional error multiplier.
 * @param ki Integral error multiplier.
 * @param kd Derivative error multiplier.
 * @param current
 * @param desired
 * @param dt Delta time since last step
 * @param eprev Previous proportional error.
 * @param eint Integral error.
 * @param plot set to non-zero to plot
 * @return Control.
 */
static float pid_step(
    float const kp,
    float const ki,
    float const kd,
    float const current,
    float const desired,
    float const dt,
    float *const eprev,
    float *const eint,
    int plot)
{
    float const e = desired - current;
    float const edot = (e - *eprev) / dt;
    *eint = *eint + (e * dt);
    if (*eint > PID_EINT_MAX)
    {
        *eint = PID_EINT_MAX;
    }
    float u = (kp * e) + (ki * *eint) + (kd * edot);
    if (u > 1.0f)
    {
        u = 1.0f;
    }
    else if (u < -1.0f)
    {
        u = -1.0f;
    }
    *eprev = e;
    
    if (plot) {
        plot_add_point(PROPORTIONAL, kp * e);
        plot_add_point(INTEGRAL, ki * *eint);
        plot_add_point(DERIVATIVE, kd * edot);
        plot_add_point(RESULT, u);
        plot_draw();
    }
    
    return u;
}

float pid_step_steer(
    float const steer_desired,
    float const steer_current,
    float const dt)
{
    return pid_step(pid_kp_steer,
                    pid_ki_steer,
                    pid_kd_steer,
                    steer_desired,
                    steer_current,
                    dt,
                    &pid_eprev_steer,
                    &pid_eint_steer,
                    1);
}

float pid_step_throttle(
    float const throttle_desired,
    float const throttle_current,
    float const dt)
{
    return pid_step(pid_kp_throttle,
                    pid_ki_throttle,
                    pid_kd_throttle,
                    throttle_desired,
                    throttle_current,
                    dt,
                    &pid_eprev_throttle,
                    &pid_eint_throttle,
                    0);
}
