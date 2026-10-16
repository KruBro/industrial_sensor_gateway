#include <sys/timerfd.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <errno.h>
#include <sys/socket.h>
#include <fcntl.h>
#include "poller.h"
#include "modbus.h"
#include "db.h"
#include "alarm.h"
#include "/home/krubro/Desktop/STM32_BARE_METAL/pots/industrial-sensor-gateway/server/hmi.h"

static volatile sig_atomic_t keep_running = 1;

void handle_signal(int sig)
{
    (void)sig;
    keep_running = 0;
}

void set_deadline(struct timespec *deadline, long ms)
{
    clock_gettime(CLOCK_MONOTONIC, deadline);
    deadline->tv_sec += ms / 1000;
    deadline->tv_nsec += (ms % 1000) * 1000000;
    if (deadline->tv_nsec >= 1000000000) {
        deadline->tv_sec++;
        deadline->tv_nsec -= 1000000000;
    }
}

int get_remaining_ms(const struct timespec *deadline)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long sec = deadline->tv_sec - now.tv_sec;
    long nsec = deadline->tv_nsec - now.tv_nsec;
    long total_ms = (sec * 1000) + (nsec / 1000000);
    return (total_ms < 0) ? 0 : (int)total_ms;
}

int poller_run(int fd, uint8_t addr, uint16_t count, unsigned interval_ms, uint16_t valve_state, const char* log_path)
{
    struct sigaction sa;
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if(sigaction(SIGINT, &sa, NULL) < 0)
    {
        perror("sigaction SIGINT");
        exit(EXIT_FAILURE);
    }

    if(sigaction(SIGTERM, &sa, NULL) < 0)
    {
        perror("sigaction SIGTERM");
        exit(EXIT_FAILURE);
    }


    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if(tfd < 0)
    {
        perror("timerfd_create");
        exit(EXIT_FAILURE);
    }

    struct itimerspec ts;
    ts.it_value.tv_sec = interval_ms / 1000;
    ts.it_value.tv_nsec = (interval_ms % 1000) * 1000000; 

    ts.it_interval.tv_sec = ts.it_value.tv_sec;
    ts.it_interval.tv_nsec = ts.it_value.tv_nsec;

    if(timerfd_settime(tfd, 0, &ts, NULL) < 0)
    {
        perror("timerfd_settime");
        close(tfd);
        exit(EXIT_FAILURE);
    }

    int server_fd = server_init();
    if(server_fd < 0)
    {
        fprintf(stderr, "Fatal : Server Initialization has failed\n");
        server_fd = -1;
    }

    struct pollfd fds[4];
    fds[0].fd = tfd;
    fds[0].events = POLLIN;

    fds[1].fd = fd;
    fds[1].events = POLLIN; 

    fds[2].fd = server_fd;
    fds[2].events = POLLIN;

    fds[3].fd = -1;
    fds[3].events = POLLIN;

    struct modbus_transaction tx;
    tx.state = STATE_IDLE;
    struct pending_request queue = {0};

    uint64_t expirations;
    struct timespec now_ts;

    uint16_t values[125];
    uint8_t exc = 0;

    ProcessDB *db = db_init("/tmp/process.db");
    if(!db)
    {
        fprintf(stderr, "Fatal : Process database mapping initialization failed\n");
        close(tfd);
        return -1;
    }

    if(alarm_init(log_path) < 0)
    {
        fprintf(stderr, "Warning Alarm System Failed to Initialize. Core loop will run without logging.\n");
    }
    printf("[Master] Initialization complete. Spawning event monitoring loop...\n");

    if(valve_state == 0xFF00 || valve_state == 0x0000)
    {
        tx.state = STATE_WAIT_REPLY;
        tx.expected_fc = MB_FC_WRITE_COIL;
        tx.addr = addr;
        tx.count = 0;
        tx.attempt = 1;
        tx.rx_len = 0;
        tx.actual_write_value = valve_state;
        uint8_t on_flag = (tx.actual_write_value == 0xFF00) ? 1 : 0;
        modbus_send_write_coil_async(fd, tx.addr, 0, on_flag);
        set_deadline(&tx.deadline, RESPONSE_TIMEOUT_MS);
    }

    while(keep_running)
    {
        int timeout_ms = -1;
        if(tx.state != STATE_IDLE)
        {
            timeout_ms = get_remaining_ms(&tx.deadline);
        }
        int ret = poll(fds, 4, timeout_ms);

        if(ret < 0)
        {
            if(errno == EINTR)
                continue; 
            perror("poll");
            break;
        }
        else if(ret == 0 && tx.state != STATE_IDLE)
        {
            if(tx.state == STATE_WAIT_REPLY)
            {
                fprintf(stderr, "attempt %d/%d: %s\n", tx.attempt, MAX_ATTEMPTS, mbm_strerror(MBM_ERR_TIMEOUT));

                db_update_failure(db, 0);
                if(tx.attempt < MAX_ATTEMPTS)
                {
                    tx.state = STATE_RETRY_PAUSE;
                    set_deadline(&tx.deadline, RETRY_DELAY_MS);
                }
                else
                {
                    tx.state = STATE_IDLE;
                }
            }
            else if(tx.state == STATE_RETRY_PAUSE)
            {
                tx.attempt++;
                tx.rx_len = 0;
                if(tx.expected_fc == MB_FC_WRITE_COIL)
                {
                    uint8_t on_flag = (tx.actual_write_value == 0xFF00) ? 1 : 0;
                    modbus_send_write_coil_async(fd, tx.addr, 0, on_flag);
                }
                else
                {
                    modbus_send_request_async(fd, tx.addr, START_REG, tx.count);
                }
                tx.state = STATE_WAIT_REPLY;
                set_deadline(&tx.deadline, RESPONSE_TIMEOUT_MS);
            }
            continue;
        }

        if(fds[1].revents & (POLLERR | POLLHUP))
        {
            fprintf(stderr, "ERROR: Modbus Connection broken or encountered an error\n");
            break;
        }

        if(fds[0].revents & POLLIN)
        {
            ssize_t s = read(tfd, &expirations, sizeof(expirations));
            if(s != sizeof(expirations))
            {
                if(errno == EINTR)
                    continue;
                perror("read");
                break;
            }

            if(tx.state == STATE_IDLE)
            {
                tx.state = STATE_WAIT_REPLY;
                tx.expected_fc = MB_FC_READ_HOLDING;
                tx.addr = addr;
                tx.count = count;
                tx.attempt = 1;
                tx.rx_len = 0;
                modbus_send_request_async(fd, tx.addr, START_REG, tx.count);
                set_deadline(&tx.deadline, RESPONSE_TIMEOUT_MS);
            }
            else
            {
                queue.has_request = 1;
                queue.is_write_coil = 0;
                queue.addr = addr;
                queue.count = count;
            }
        }

        if(fds[1].revents & POLLIN)
        {
            uint8_t temp;
            ssize_t n = read(fd, &temp, 1);
            if(n > 0 && tx.state == STATE_WAIT_REPLY)
            {
                if(tx.rx_len < MAX_FRAME_LEN)
                {
                    tx.rx_buf[tx.rx_len++] = temp;
                }

                mbm_status_t st = modbus_parse_buffer_async(tx.rx_buf, tx.rx_len, tx.count, tx.addr, values, &exc, tx.expected_fc);
                if(st == MBM_OK)
                {
                    db_update_success(db, values, tx.count);
                    if(tx.expected_fc == MB_FC_READ_HOLDING)
                    {
                        if(tx.count == 4)
                        {
                            printf("P=%u.%u bar F=%u.%u L/min T=%u.%u Valve: %s\n", 
                                    values[0] / 10, values[0] % 10,
                                    values[1] / 10, values[1] % 10,
                                    values[2] / 10, values[2] % 10,
                                    (values[3] == 1) ? "OPEN" : "CLOSED");
                        }
                        else
                        {
                            for(unsigned int i = 0; i < tx.count; i++)
                            {
                                printf("reg[%u] = %u\n", i, values[i]);
                            }
                        }
                        alarm_check(values);
                        clock_gettime(CLOCK_REALTIME, &now_ts);
                        printf("[%ld.%09ld] 1\n", now_ts.tv_sec, now_ts.tv_nsec);
                        fflush(stdout);
                    }
                    else if(tx.expected_fc == MB_FC_WRITE_COIL)
                    {
                        printf("[Master] Valve action write coil verification successful!\n");
                        fflush(stdout);
                    }

                    if(tx.attempt > 1)
                    {
                        fprintf(stderr, "note : succeeded on attempt %d : %d\n", tx.attempt, MAX_ATTEMPTS);
                    }

                    if(queue.has_request)
                    {
                        tx.state = STATE_WAIT_REPLY;
                        tx.attempt = 1;
                        tx.rx_len = 0;
                        tx.addr = queue.addr;
                        if(queue.is_write_coil)
                        {
                            tx.expected_fc = MB_FC_WRITE_COIL;
                            tx.count = 0;
                            tx.actual_write_value = queue.write_value;
                            uint8_t on_flag = (tx.actual_write_value == 0xFF00) ? 1 : 0;
                            modbus_send_write_coil_async(fd, tx.addr, 0, on_flag);
                        }
                        else
                        {
                            tx.expected_fc = MB_FC_READ_HOLDING;
                            tx.count = queue.count;
                            modbus_send_request_async(fd, tx.addr, START_REG, tx.count);
                        }
                        queue.has_request = 0;
                        set_deadline(&tx.deadline, RESPONSE_TIMEOUT_MS);
                    }
                    else
                    {
                        tx.state = STATE_IDLE;
                    }
                }
                else if(st == MBM_ERR_EXCEPTION)
                {
                    db_update_failure(db, 0);
                    fprintf(stderr, "ERROR : %s : code : %u (%s)\n", mbm_strerror(st), exc, exception_name(exc));
                    tx.state = STATE_IDLE;
                }
                else if(st != MBM_INCOMPLETE)
                {
                    fprintf(stderr, "attempt : %d:%d: %s\n", tx.attempt, MAX_ATTEMPTS, mbm_strerror(st));

                    if(st == MBM_ERR_CRC)
                        db_update_failure(db, 1);
                    else
                        db_update_failure(db, 0);

                    if(tx.attempt < MAX_ATTEMPTS && is_retryable(st))
                    {
                        tx.state = STATE_RETRY_PAUSE;
                        set_deadline(&tx.deadline, RETRY_DELAY_MS);
                    }
                    else
                    {
                        tx.state = STATE_IDLE;
                    }
                }
            }
        }
        
        if(fds[2].revents & POLLIN)
        {
            int new_client = accept(server_fd, NULL, NULL);
            if(new_client >= 0)
            {
                int flags = fcntl(new_client, F_GETFL, 0);
                fcntl(new_client, F_SETFL, flags | O_NONBLOCK, 0);

                if(fds[3].fd >= 0)
                {
                    close(fds[3].fd);
                }

                fds[3].fd = new_client;
                hmi_reset_client_buffer();
            }
        }

        if(fds[3].fd >= 0 && (fds[3].revents & (POLLIN | POLLHUP | POLLERR)))
        {
            if(server_handle_request(fds[3].fd, db, &queue) < 0)
            {
                close(fds[3].fd);
                fds[3].fd = -1;
            }
        }
    }

    printf("\nLoop stopped. Cleaning up and exiting safely.\n");
    hmi_shutdown(server_fd, fds[3].fd);
    alarm_shutdown();
    db_close(db);
    close(tfd);
    return 0;
}
