#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "alarm.h"

static AlarmChannel channels[] = {
    { "PRESS",    3200,   64,          0 },  
    { "FLOW",     8500,   250,         0 },  
    { "TEMP",     1200,   30,          0 }   
};

static int alarm_pipe_wr_fd = -1;
static pid_t child_pid = -1;
static uint32_t dropped_count = 0;
static volatile sig_atomic_t child_reaped = 0;
static int death_warned = 0;

void handle_sigchld(int sig)
{
    (void)sig; 
    int saved_errno = errno;
    while(waitpid(-1, NULL, WNOHANG) > 0);
    child_reaped = 1;
    child_pid = -1;
    errno = saved_errno;
}

static void close_inherited_fds(int keep_fd)
{
    long max_fd = sysconf(_SC_OPEN_MAX);
    if(max_fd < 0)
    {
        max_fd = 1024;
    }

    for(int fd = 3; fd < (int)max_fd; fd++)
    {
        if(fd != keep_fd)
        {
            close(fd);
        }
    }
}

static ssize_t read_full(int fd, void *buf, size_t count) {
    uint8_t *ptr = (uint8_t *)buf;
    size_t total = 0;
    while (total < count) {
        ssize_t n = read(fd, ptr + total, count - total);
        if (n == 0) return total; 
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1; 
        }
        total += n;
    }
    return total;
}

static const char *severity_word(uint8_t sev) {
    switch (sev) {
        case 1:  return "WARN";
        case 2:  return "HIGH";
        case 3:  return "CRIT";
        default: return "UNKN";
    }
}

static void child_logger_loop(int pipe_rd_fd, int log_fd)
{
    AlarmEvent ev;

    if(pipe_rd_fd < 0)
    {
        perror("pipe pipe_read_fd");
        return;
    }

    while(1)
    {
        ssize_t read_size = read_full(pipe_rd_fd, &ev, sizeof(AlarmEvent));
        char line[128];
        if(read_size == 0)
            break;
        
        if(read_size < 0)
        {
            perror("alarm child : read error");
            break;
        }

        if((size_t)read_size < sizeof(AlarmEvent))
        {
            fprintf(stderr, "alarm child: partial event (%zd bytes), discarding\n", read_size);
            break;
        }

        ev.tag[sizeof(ev.tag) - 1] = '\0';

        int len = snprintf(line, sizeof(line), "%u %s %s %u %u\n",
                    ev.timestamp,
                    severity_word(ev.severity),
                    ev.tag,
                    ev.value,
                    ev.limit);
        
        if(log_fd >= 0 && len > 0)
        {
            if((size_t)len >= sizeof(line))
            {
                len = sizeof(line) - 1;
            }
            ssize_t written = write(log_fd, line, (size_t)len);
            (void)written;
        }
    }

    close(log_fd);
    close(pipe_rd_fd);
}

int alarm_init(const char *log_path)
{
    struct sigaction sa_child;
    sa_child.sa_handler = handle_sigchld;
    sigemptyset(&sa_child.sa_mask);
    sa_child.sa_flags = SA_RESTART | SA_NOCLDSTOP;

    if(sigaction(SIGCHLD, &sa_child, NULL) < 0)
    {
        perror("sigaction SIGCHLD");
        exit(EXIT_FAILURE);
    }

    signal(SIGPIPE, SIG_IGN);

    int pfd[2];
    if(pipe(pfd) < 0)
    {
        perror("pipe");
        return -1;
    }

    child_reaped = 0;
    death_warned = 0;

    int ret = fork();
    if(ret < 0)
    {
        perror("fork");
        close(pfd[0]);
        close(pfd[1]);
        return -1;
    }
    else if(ret == 0)
    {
        close(pfd[1]); 
        close_inherited_fds(pfd[0]);
        int log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if(log_fd < 0)
        {
            perror("open ->log_fd");
            close(pfd[0]);
            _exit(EXIT_FAILURE);
        }

        child_logger_loop(pfd[0], log_fd);
        _exit(EXIT_SUCCESS);
    }
    else if(ret > 0)
    {
        child_pid = ret;
        close(pfd[0]); 
        alarm_pipe_wr_fd = pfd[1]; 
        int flags = fcntl(alarm_pipe_wr_fd, F_GETFL, 0);
        fcntl(alarm_pipe_wr_fd, F_SETFL, flags | O_NONBLOCK); 
    }

    return 0;
}

static uint8_t determine_severity(uint32_t overshoot, uint16_t limit)
{
    if(overshoot > (uint32_t)(limit * 25) / 100)
    {
        return 3;
    }
    if(overshoot > (uint32_t)(limit * 10) / 100)
    {
        return 2;
    }
    return 1;
}

void alarm_check(const uint16_t *regs)
{
    if(alarm_pipe_wr_fd < 0 || regs == NULL)
        return;

    if (child_reaped && !death_warned) {
        fprintf(stderr, "[Alarm Engine] Warning: Logging child process died unexpectedly!\n");
        death_warned = 1;
    }

    for(size_t i = 0; i < NUM_CHANNELS; i++)
    {
        uint16_t current_val = regs[i];
        uint16_t limit = channels[i].limit;
        uint16_t hyst = channels[i].hysteresis;
        
        if(channels[i].in_alarm == 0)
        {
            if(current_val > limit)
            {
                channels[i].in_alarm = 1;

                AlarmEvent ev;
                memset(&ev, 0, sizeof(AlarmEvent));

                strncpy(ev.tag, channels[i].tag, sizeof(ev.tag) - 1);
                ev.limit = limit;
                ev.value = current_val;
                ev.timestamp = (uint32_t)time(NULL);
                uint32_t overshoot = current_val - limit;
                ev.severity = determine_severity(overshoot, limit);

                ssize_t n = write(alarm_pipe_wr_fd, &ev, sizeof(AlarmEvent));
                if(n < 0)
                {
                    if(errno == EAGAIN || errno == EWOULDBLOCK)
                    {
                        dropped_count++;
                        fprintf(stderr, "[Alarm Engine] Warning: Pipe buffer full, dropped alarm event.\n");
                    }
                    else
                    {
                        perror("[Alarm Engine] Write to child logger failed");
                    }
                }
            }
        }
        else if(channels[i].in_alarm && current_val < (limit - hyst))
        {
            channels[i].in_alarm = 0;
        }
    }
}

void alarm_shutdown(void)
{
    if(alarm_pipe_wr_fd >= 0)
    {
        close(alarm_pipe_wr_fd);
        alarm_pipe_wr_fd = -1;
    }
}
