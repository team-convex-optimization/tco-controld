#include <stdlib.h>
#include <stdio.h>

#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

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
    shmem_control_open = 1;
    /* START: Critical section */
    shmem_control->ch[0].active = 0;
    shmem_control->ch[0].pulse_frac = 0;
    shmem_control->ch[1].active = 0;
    shmem_control->ch[1].pulse_frac = 0;
    /* END: Critical section */
    if (sem_post(shmem_sem_control) == -1)
    {
        log_error("sem_post: %s", strerror(errno));
        exit(0); /* XXX: This can potentially lead to deadlocks. */
    }
    shmem_control_open = 0;
    exit(0);
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
    float steering_angle = 0;
    float throttle = 0;

    uint16_t waypts[TCO_PLAN_WAYPTS_N][2] = {0};
    uint8_t waypts_valid = 0;
    while (1)
    {
        if (sem_wait(shmem_sem_plan) == -1)
        {
            log_error("sem_wait: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_plan_open = 1;
        /* START: Critical section */
        waypts_valid = shmem_plan->valid;
        if (waypts_valid)
        {
            memcpy(waypts, shmem_plan->waypts, TCO_PLAN_WAYPTS_N * 2 * sizeof(shmem_plan->waypts[0][0]));
        }
        /* END: Critical section */
        if (sem_post(shmem_sem_plan) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_plan_open = 0;

        if (waypts_valid)
        {
            if (waypts[0][0] >= TCO_FRAME_WIDTH || waypts[0][1] >= TCO_FRAME_HEIGHT)
            {
                steering_angle = 0.0;
                throttle = 0.0;
            }
            else
            {
                steering_angle = (((waypts[0][0] / (float)TCO_FRAME_WIDTH) * 2.0f) - 1.0f);
                steering_angle *= 1.3;
                if (steering_angle > 1.0f)
                {
                    steering_angle = 1.0f;
                }
                printf("Steer: %f\n", steering_angle);
                throttle = 0.01;
            }

            if (sem_wait(shmem_sem_control) == -1)
            {
                log_error("sem_wait: %s", strerror(errno));
                goto handle_error_and_exit;
            }
            shmem_control_open = 1;
            /* START: Critical section */
            shmem_control->valid = 1;
            shmem_control->ch[0].active = 1;
            shmem_control->ch[0].pulse_frac = (throttle + 1.0f) / 2.0f;

            shmem_control->ch[1].active = 1;
            shmem_control->ch[1].pulse_frac = (steering_angle + 1.0f) / 2.0f;
            /* END: Critical section */
            if (sem_post(shmem_sem_control) == -1)
            {
                log_error("sem_post: %s", strerror(errno));
                goto handle_error_and_exit;
            }
            shmem_control_open = 0;
        }
    }

    return 0;

handle_error_and_exit:
    log_deinit(); /* It doesn't matter if this fails */
    return EXIT_FAILURE;
}
