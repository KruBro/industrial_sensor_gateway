#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "master.h"
#include "main_helper.h"

#define DEFAULT_PORT          "/dev/tnt1"
#define DEFAULT_LOG           "/tmp/gateway_alarms.log"
#define DEFAULT_ADDR          0x01
#define DEFAULT_COUNT         4
#define RUN_PAUSE_MS          100
#define MAX_RUNS              1000

typedef struct {
    unsigned runs;
    unsigned ok_first_try;
    unsigned ok_after_retry;
    unsigned failed;
    unsigned total_attempts;
    unsigned fail_by_status[MBM_STATUS_COUNT];
} mbm_stats_t;

static void stats_record(mbm_stats_t *s, mbm_status_t st, int attempts)
{
    s->runs++;
    if (attempts > 0)
    {
        s->total_attempts += (unsigned)attempts;
    }

    if (st == MBM_OK)
    {
        if (attempts == 1)
        {
            s->ok_first_try++;
        }
        else
        {
            s->ok_after_retry++;
        }
    }
    else
    {
        s->failed++;
        if ((int)st >= 0 && (int)st < MBM_STATUS_COUNT)
        {
            s->fail_by_status[st]++;
        }
    }
}

static void stats_print(const mbm_stats_t *s)
{
    printf("runs=%u  ok_first_try=%u  ok_after_retry=%u  failed=%u  attempts=%u\n",
           s->runs, s->ok_first_try, s->ok_after_retry, s->failed, s->total_attempts);

    if (s->failed > 0)
    {
        printf("failures:");
        for (int st = 0; st < MBM_STATUS_COUNT; st++)
        {
            if (s->fail_by_status[st] > 0)
            {
                printf("  %s=%u", mbm_strerror((mbm_status_t)st), s->fail_by_status[st]);
            }
        }
        printf("\n");
    }
}

int run_single(int fd, uint8_t addr, uint16_t count)
{
    uint16_t values[125];
    uint8_t exc = 0;
    int used = 0;

    mbm_status_t st = modbus_read_registers_retry(fd, addr, START_REG, count, values,
                                                  &exc, MAX_ATTEMPTS, &used);

    if (st == MBM_ERR_EXCEPTION)
    {
        fprintf(stderr, "ERROR: %s: code %u (%s)\n", mbm_strerror(st), exc, exception_name(exc));
        return 1;
    }
    if (st != MBM_OK)
    {
        fprintf(stderr, "ERROR: %s\n", mbm_strerror(st));
        return 1;
    }

    if (count == 3)
    {
        printf("P=%u.%u bar  F=%u.%u L/min  T=%u.%u C\n",
               values[0] / 10, values[0] % 10,
               values[1] / 10, values[1] % 10,
               values[2] / 10, values[2] % 10);
    }
    else
    {
        for (unsigned i = 0; i < count; i++)
        {
            printf("reg[%u] = %u\n", i, values[i]);
        }
    }

    if (used > 1)
    {
        fprintf(stderr, "note: succeeded on attempt %d of %d\n", used, MAX_ATTEMPTS);
    }

    return 0;
}

int run_counting(int fd, uint8_t addr, uint16_t count, unsigned runs)
{
    mbm_stats_t stats;
    uint16_t values[125];
    uint8_t exc = 0;
    int aborted = 0;

    memset(&stats, 0, sizeof(stats));

    for (unsigned r = 0; r < runs; r++)
    {
        int used = 0;

        mbm_status_t st = modbus_read_registers_retry(fd, addr, START_REG, count, values,
                                                      &exc, MAX_ATTEMPTS, &used);
        stats_record(&stats, st, used);

        if (st == MBM_ERR_ARGS || st == MBM_ERR_IO)
        {
            fprintf(stderr, "aborting: %s\n", mbm_strerror(st));
            aborted = 1;
            break;
        }

        if (r + 1 < runs)
        {
            usleep(RUN_PAUSE_MS * 1000U);
        }
    }

    stats_print(&stats);

    return (stats.failed == 0 && !aborted) ? 0 : 1;
}

#include <string.h>

// Inside your arguments struct definition, make sure valve_value can hold a signed integer:
// int valve_value; // -1 = No Action, 0x0000 = Close, 0xFF00 = Open

int parse_args(int argc, char *argv[], cli_args_t *args)
{
    #define IS_VALID_ARG(index) (argc > (index) && strcmp(argv[index], "NULL") != 0 && strcmp(argv[index], "-") != 0)

    args->port  = IS_VALID_ARG(1) ? argv[1] : DEFAULT_PORT;
    args->count = IS_VALID_ARG(2) ? strtoul(argv[2], NULL, 0) : DEFAULT_COUNT;
    args->addr  = IS_VALID_ARG(3) ? strtoul(argv[3], NULL, 0) : DEFAULT_ADDR;
    args->runs  = IS_VALID_ARG(4) ? strtoul(argv[4], NULL, 0) : 0; 

    args->valve_value = IS_VALID_ARG(5) ? (int)strtoul(argv[5], NULL, 0) : -1;

    if (args->count < 1 || args->count > 125 || args->addr > 247 || args->runs > MAX_RUNS)
    {
        fprintf(stderr,
                "usage: %s [port] [count] [slave_addr] [runs] [valve_value: 0x0000|0xFF00]\n", argv[0]);
        return -1;
    }

    args->log_path = IS_VALID_ARG(6) ? argv[6] : DEFAULT_LOG; 

    return 0;
}


