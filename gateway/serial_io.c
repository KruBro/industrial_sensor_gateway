#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include "serial_io.h"


ssize_t send_serial_frame(int fd, const uint8_t *buffer, ssize_t len, unsigned int sim_byte_delay_us)
{
    if (fd < 0 || !buffer || len <= 0)
    {
        return -1;
    }

    if (sim_byte_delay_us == 0)
    {
        ssize_t total_written = 0;

        while (total_written < len)
        {
            ssize_t n = write(fd, buffer + total_written, (size_t)(len - total_written));
            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                perror("Serial whole-frame write failed");
                return -1;
            }

            total_written += n;
        }

        return total_written;
    }

    for (ssize_t i = 0; i < len; )
    {
        ssize_t n = write(fd, &buffer[i], 1);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("Serial byte-by-byte write failed");
            return -1;
        }

        if (n == 1)
        {
            i++;
            if (i < len)
            {
                usleep(sim_byte_delay_us);
            }
        }
    }

    return len;
}

ssize_t read_serial_frame(int fd, uint8_t *buf, ssize_t target_len, int timeout_ms)
{
    if (fd < 0 || !buf || target_len <= 0)
    {
        return -1;
    }

    ssize_t total_bytes_read = 0;

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    while (total_bytes_read < target_len)
    {
        clock_gettime(CLOCK_MONOTONIC, &current_time);

        long long elapsed_ms =
            ((long long)(current_time.tv_sec - start_time.tv_sec) * 1000000000LL +
             (current_time.tv_nsec - start_time.tv_nsec)) / 1000000LL;

        long long time_left = timeout_ms - elapsed_ms;
        if (time_left <= 0)
        {
            break;
        }

        int poll_result = poll(&pfd, 1, (int)time_left);
        if (poll_result < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            perror("poll");
            return -1;
        }
        if (poll_result == 0)
        {
            break;
        }

        if (pfd.revents & (POLLERR | POLLNVAL))
        {
            fprintf(stderr, "Serial port error (revents=0x%x)\n", (unsigned)pfd.revents);
            return -1;
        }

        if (pfd.revents & POLLIN)
        {
            ssize_t n = read(fd, buf + total_bytes_read, (size_t)(target_len - total_bytes_read));
            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                perror("read");
                return -1;
            }
            if (n == 0)
            {
                fprintf(stderr, "Serial device disconnected unexpectedly\n");
                return -1;
            }

            total_bytes_read += n;
        }
        else if (pfd.revents & POLLHUP)
        {
            fprintf(stderr, "Serial device hung up\n");
            return -1;
        }
    }

    return total_bytes_read;
}

int open_port(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror("open");
        return -1;
    }

    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CREAD | CLOCAL);

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}
