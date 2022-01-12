#ifndef PLOT_H_
#define PLOT_H_
#define FILENAME "pid_data.txt"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

struct plot_state {
    unsigned int step;
    FILE *file;
    uint8_t fail;
    float line[4]; /* Stores the data to write on each line (PID) values */
};

enum DATA {
    PROPORTIONAL    = 0,
    INTEGRAL        = 1,
    DERIVATIVE      = 2,
    RESULT          = 3,
};

static struct plot_state state;

/**
 * @brief instatiate the plot instance. Will override any existing file(s)
 */
int plot_init();

/**
 * @brief add a point to FILENAME at data_point of current step. 
 * @param d is the field to fill
 * @param value is the value to write
 * @note step is kept track of as the number of times `plot_draw()` is called. 
 */
void plot_add_point(enum DATA d, float value);

/**
 * @brief draw the last step
 */
void plot_draw(void);

/**
 * @brief cleanup 
 */
void plot_deinit(void);


#endif /* PLOT_H_ */