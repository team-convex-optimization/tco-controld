#include <stdlib.h>
#include <stdio.h>

#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <math.h>

#include "tco_libd.h"
#include "tco_shmem.h"

#include "pid.h"

int log_level = LOG_DEBUG | LOG_ERROR | LOG_INFO;

static struct tco_shmem_data_control *shmem_control;
static struct tco_shmem_data_plan *shmem_plan;
static sem_t *shmem_sem_control;
static sem_t *shmem_sem_plan;
static uint8_t shmem_control_open = 0;
static uint8_t shmem_plan_open = 0;

/**
 * @brief Handler for signals. This ensures that deadlocks in shmems do not occur and  when
 * clontrold is closed, control shmem is reset.
 * @param sig Signal number. This is ignored since this handler is registered for the right signals already.
 */
static void handle_signals(int sig)
{
    if (shmem_control_open)
    {
        if (sem_post(shmem_sem_control) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
        }
    }
    if (shmem_plan_open)
    {
        if (sem_post(shmem_sem_plan) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
        }
    }

    if (sem_wait(shmem_sem_control) == -1)
    {
        log_error("sem_wait: %s", strerror(errno));
        exit(0);
    }
    /* START: Critical section */
    shmem_control_open = 1;
    shmem_control->ch[0].active = 0;
    shmem_control->ch[0].pulse_frac = 0;
    shmem_control->ch[1].active = 0;
    shmem_control->ch[1].pulse_frac = 0;
    /* END: Critical section */
    if (sem_post(shmem_sem_control) == -1)
    {
        log_error("sem_post: %s", strerror(errno));
        // exit(0); /* XXX: This can potentially lead to deadlocks. */
    }
    shmem_control_open = 0;
    exit(0);
}

/**
 * @brief apply sigmoid activation on @p x with curvature defined with @p sense
 * @param x input
 * @param sense the avg gradient of sigmoid function (factor of x)
 * @result A function that follow a sigmoid function
 * @note LaTeX : `f\left(x\right)\:=\:\left(\frac{1}{1+e^{-sx}}-.5\right)\cdot 2`
*/
float sigmoid_acvitvation(float x, float sense) {
	float e = 2.71828; /* Eulers Number */
	float denom = 1 + powf(e, (sense * x) * -1); /* Defined in `math.h` stdlib */
	return ((1/denom) - 0.5) * 2;
}

int main(int argc, char const *argv[])
{
    struct sigaction sa;
    sa.sa_handler = handle_signals;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (log_init("controld", "./log.txt") != 0)
    {
        printf("Failed to initialize the logger");
        return EXIT_FAILURE;
    }

    if (shmem_map(TCO_SHMEM_NAME_CONTROL, TCO_SHMEM_SIZE_CONTROL, TCO_SHMEM_NAME_SEM_CONTROL, O_RDWR, (void **)&shmem_control, &shmem_sem_control) != 0)
    {
        log_error("Failed to map control shmem into process memory");
        goto handle_error_and_exit;
    }
    if (shmem_map(TCO_SHMEM_NAME_PLAN, TCO_SHMEM_SIZE_PLAN, TCO_SHMEM_NAME_SEM_PLAN, O_RDONLY, (void **)&shmem_plan, &shmem_sem_plan) != 0)
    {
        log_error("Failed to map planning shmem into process memory");
        goto handle_error_and_exit;
    }

    /* Between -1.0 and 1.0 */
    float steer_frac_raw = 0;
    float throttle_frac_raw = 0;

    float target_pos = 0;
    float target_speed = 0;
    uint32_t frame_id = 0;
    uint32_t frame_id_last = 0;
    while (1)
    {
        if (sem_wait(shmem_sem_plan) == -1)
        {
            log_error("sem_wait: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        /* START: Critical section */
        shmem_plan_open = 1;
        target_pos = shmem_plan->target_pos;
        target_speed = shmem_plan->target_speed;
        frame_id = shmem_plan->frame_id;
        /* END: Critical section */
        if (sem_post(shmem_sem_plan) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_plan_open = 0;

        if (frame_id == frame_id_last)
        {
            continue;
        }
        frame_id_last = frame_id;

        /* At 22fps, dt is 33 milliseconds. */
        /* TODO: Measure the time between frames instead of relying on a constant. */
        steer_frac_raw = -pid_step_steer(target_pos, 0.0f, (1.0f / 21.0f));
        throttle_frac_raw = -pid_step_throttle(target_speed, 0.0f, (1.0f / 21.0f)); //1 - fabs(sigmoid_acvitvation(steer_frac_raw, 3.0f)); //TODO: Use PID
        throttle_frac_raw *= 0.3;
        printf("steer %f and throttle %f(%f)\n", steer_frac_raw, throttle_frac_raw, target_speed);

        if (sem_wait(shmem_sem_control) == -1)
        {
            log_error("sem_wait: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        /* START: Critical section */
        shmem_control_open = 1;
        shmem_control->ch[0].active = 1;
        shmem_control->ch[0].pulse_frac = (throttle_frac_raw + 1.0f) / 2.0f;

        shmem_control->ch[1].active = 1;
        shmem_control->ch[1].pulse_frac = (steer_frac_raw + 1.0f) / 2.0f;
        /* END: Critical section */
        if (sem_post(shmem_sem_control) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_control_open = 0;
    }

    return 0;

handle_error_and_exit:
    log_deinit(); /* It doesn't matter if this fails */
    return EXIT_FAILURE;
}
