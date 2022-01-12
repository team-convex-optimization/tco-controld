#include "plot.h"

int plot_init() {
    state.step = 0;
    state.fail = 0;

    /* Open file */
    state.file = fopen(FILENAME, "w");
    if (state.file == NULL) {
        fprintf(stderr, "Failed to instantiate the PID_value logging \n");
        state.fail = 1;
        return -1;
    }
    return 0;
}

void plot_add_point(enum DATA d, float value) {
    if (state.fail) return;
    state.line[d] = value;
}

void plot_draw(void) {
    if (state.fail) return;
    char to_write[1024];
    int ret = fwrite(to_write, sizeof(float), 4, state.file);
    if (ret == 0) {
        fprintf(stderr, "Could not write to file \n");
        state.fail = 1;
    }
}

void plot_deinit(void) {
    fclose(state.file);
}