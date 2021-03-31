#include <stdlib.h>
#include <stdio.h>

#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "tco_libd.h"
#include "tco_shmem.h"

int log_level = LOG_DEBUG | LOG_ERROR | LOG_INFO;

int main(int argc, char const *argv[])
{
    if (log_init("controld", "./log.txt") != 0)
    {
        printf("Failed to initialize the logger");
        return EXIT_FAILURE;
    }

    struct tco_shmem_data_control *shmem_control;
    struct tco_shmem_data_plan *shmem_plan;
    sem_t *shmem_sem_control;
    sem_t *shmem_sem_plan;
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
    }

    return 0;

handle_error_and_exit:
    log_deinit(); /* It doesn't matter if this fails */
    return EXIT_FAILURE;
}
