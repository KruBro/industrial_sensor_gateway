#ifndef MAIN_HELPER_H
#define MAIN_HELPER_H

#include <stdint.h>

typedef struct {
    const char   *port;
    unsigned long count;
    unsigned long addr;
    unsigned long runs;
    u_int16_t valve_value;
    const char   *log_path;
} cli_args_t;

/* Fills *args from the command line. Prints the usage text and returns -1
 * if an argument is out of range, otherwise returns 0. */
int parse_args(int argc, char *argv[], cli_args_t *args);

/* One read with retries; prints the register values. Returns 0 on success. */
int run_single(int fd, uint8_t addr, uint16_t count);

/* Repeats the read `runs` times and prints a success summary.
 * Returns 0 only if every run succeeded. */
int run_counting(int fd, uint8_t addr, uint16_t count, unsigned runs);

#endif
