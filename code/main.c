#include <stdlib.h>
#include <stdio.h>

#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include "tco_libd.h"
#include "tco_shmem.h"

#include "pid.h"

#ifdef __ARM_ARCH
    #define EMERGENCY_STOP_DIST (-100.0f) /* CM */
    #define MAX_RPM 700.0f
#else
    #define EMERGENCY_STOP_DIST (-1.0f) /* CM */
    #define MAX_RPM 700.0f
#endif

#define NS_TO_S 1000000000.0f

int log_level = LOG_DEBUG | LOG_ERROR | LOG_INFO;

static struct tco_shmem_data_control *shmem_control;
static struct tco_shmem_data_plan *shmem_plan;
static struct tco_shmem_data_sensor *shmem_sensor;
static sem_t *shmem_sem_control;
static sem_t *shmem_sem_plan;
static sem_t *shmem_sem_sensor;
static uint8_t shmem_control_open = 0;
static uint8_t shmem_plan_open = 0;
static uint8_t shmem_sensor_open = 0;
struct timespec timer_plan;
struct timespec timer_sensor;

float min(float a, float b) {
    return (a < b) ? a : b;
}

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
    if (shmem_control_open)
    {
        if (sem_post(shmem_sem_control) == -1)
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
    shmem_control->ch[0].active = 1;
    shmem_control->ch[0].pulse_frac = 0.0f;
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
 * @brief Get the elapsed time since last call. Global var @p timer keep track of last time.
 * Will update @p timer on each call.
 * @return float with elapsed time in seconds
 */
static float get_elapsed_time(struct timespec *time_old) {
    struct timespec time_new;
    clock_gettime(_POSIX_MONOTONIC_CLOCK, &time_new);
    float res = (time_new.tv_nsec - time_old->tv_nsec)/NS_TO_S;
    time_old->tv_nsec = time_new.tv_nsec; /* Only need to update ns as s is never used. This gives more precision */
    return res;
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
        printf("Failed to initialize the logger\n");
        return EXIT_FAILURE;
    }

    #ifdef __ARM_ARCH
    log_info("Detected ARM Architecture... Loading target specification.");
    #else
    log_info("Detected non-ARM Architecture... Loading sim specification.");
    #endif

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
    if (shmem_map(TCO_SHMEM_NAME_SENSOR, TCO_SHMEM_SIZE_SENSOR, TCO_SHMEM_NAME_SEM_SENSOR, O_RDONLY, (void **)&shmem_sensor, &shmem_sem_sensor) != 0)
    {
        log_error("Failed to map sensor shmem into process memory");
        goto handle_error_and_exit;
    }

    /* Between -1.0 and 1.0 */
    float steer_frac_raw = 0;
    float throttle_frac_raw = 0;

    float target_pos = 0;
    float target_speed = 0;
    uint32_t frame_id = 0;
    uint32_t sensor_step = 0;
    uint32_t frame_id_last = 0;
    uint32_t sensor_step_last = 0;
    double us_1 = 0;
    double rpm = 0;

    /* Start time */
    float time_elapsed_plan = 0.0f;
    float time_elapsed_sensor = 0.0f;
    clock_gettime(_POSIX_MONOTONIC_CLOCK, &timer_sensor);
    clock_gettime(_POSIX_MONOTONIC_CLOCK, &timer_plan);

    while (1)
    {
        if (sem_wait(shmem_sem_plan) == -1)         /* Get plan data */
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

        if (sem_wait(shmem_sem_sensor) == -1)           /* Get sensor data */
        {
            log_error("sem_wait: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        /* START: Critical section */
        shmem_sensor_open = 1;
        sensor_step = shmem_sensor->time_step;
        us_1 = shmem_sensor->ultrasound_left;
        rpm = shmem_sensor->hall_effect_rpm;
        /* END: Critical section */
        if (sem_post(shmem_sem_sensor) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_sensor_open = 0;

        if (frame_id != frame_id_last) /* Only update steering if we got new image */
        {
            time_elapsed_plan = get_elapsed_time(&timer_plan); /* Get time elapsed */
            steer_frac_raw = -pid_step_steer(target_pos, 0.0f, time_elapsed_plan);
            frame_id_last = frame_id;
        }

        if (sensor_step != sensor_step_last) { /* Update the motor RPM everytime we get new reading */
            time_elapsed_sensor = get_elapsed_time(&timer_sensor); /* Get time elapsed */

            float desired_rpm = ((target_speed + 1)/2.0f) * MAX_RPM;
	    float desired_rpm_error = ((desired_rpm - (float)rpm) / MAX_RPM);
            throttle_frac_raw = -pid_step_throttle(desired_rpm_error, 0.0f, time_elapsed_sensor);
            sensor_step_last = sensor_step;
        }

        if (us_1 < EMERGENCY_STOP_DIST) { /* Emergency stop */
            printf("EMERGENCY STOP!\n");
            throttle_frac_raw = -1.0f;
        }
        if (sem_wait(shmem_sem_control) == -1) /* Write to control shmem for actuation */
        {
            log_error("sem_wait: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        /* START: Critical section */
        shmem_control_open = 1;
        shmem_control->ch[0].active = 1;
        shmem_control->ch[0].pulse_frac = min((throttle_frac_raw + 1.0f) / 2.0f, 0.7f);

        shmem_control->ch[1].active = 1;
        shmem_control->ch[1].pulse_frac = (steer_frac_raw + 1.0f) / 2.0f;
        /* END: Critical section */
        if (sem_post(shmem_sem_control) == -1)
        {
            log_error("sem_post: %s", strerror(errno));
            goto handle_error_and_exit;
        }
        shmem_control_open = 0;
	    usleep(100);
    }
	
    return 0;

handle_error_and_exit:
    log_deinit(); /* It doesn't matter if this fails */
    return EXIT_FAILURE;
}
