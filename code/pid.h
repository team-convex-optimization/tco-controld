#ifndef _PID_H_
#define _PID_H_

#include "plot.h"

/**
 * @brief Step the PID controller for steering.
 * @param steer_desired Between -1.0 (all the way left) and 1.0 (all the way right)
 * @param steer_current Between -1.0 (all the way left) and 1.0 (all the way right)
 * @param dt Delta time since last step.
 * @return Control.
 */
float pid_step_steer(
    float const steer_desired,
    float const steer_current,
    float const dt);

/**
 * @brief Step the PID controller for throttle.
 * @param steer_desired Between -1.0 (max brake) and 1.0 (max throttle)
 * @param steer_current Between -1.0 (max brake) and 1.0 (max throttle)
 * @param dt Delta time since last step.
 * @return Control.
 */
float pid_step_throttle(
    float const throttle_desired,
    float const throttle_current,
    float const dt);

#endif /* _PID_H_ */