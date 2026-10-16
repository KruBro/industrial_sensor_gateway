#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <stdint.h>
#include <sys/types.h>

ssize_t send_serial_frame(int fd, const uint8_t *buffer, ssize_t len, unsigned int sim_byte_delay_us);
ssize_t read_serial_frame(int fd, uint8_t *buf, ssize_t target_len, int timeout_ms);

/* Opens and configures a serial port: 9600 8N1, raw, no flow control. Returns fd or -1. */
int open_port(const char *path);

#endif