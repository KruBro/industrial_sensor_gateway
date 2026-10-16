#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include "db.h"

int main(int argc, char *argv[])
{
    const char *db_path = (argc > 1) ? argv[1] : "/tmp/process.db";

    int fd = open(db_path, O_RDONLY);
    if(fd < 0)
    {
        perror("open");
        return EXIT_FAILURE;
    }

    const ProcessDB *db_ptr = (const ProcessDB *)mmap(NULL, sizeof(ProcessDB), PROT_READ, MAP_SHARED, fd, 0);

    close(fd);

    if(db_ptr == MAP_FAILED)
    {
        perror("mmap");
        return EXIT_FAILURE;
    }

    printf("--- Monitoring Mapped Process Database %s ---\n", db_path);
    printf("Press Ctrl + C to Exit\n\n");

    ProcessDB local_copy;
    uint32_t start_seq;

    while(1)
    {
        do
        {
            start_seq = db_ptr->seq_count;

            memcpy(&local_copy, db_ptr, sizeof(ProcessDB));
        } while ((start_seq % 2 != 0) || (start_seq != db_ptr->seq_count));
        
        char time_str[32] = "NEVER";
        if(local_copy.last_poll_time > 0)
        {
            struct tm *tm_info = localtime(&local_copy.last_poll_time);
            if(tm_info)
            {
                strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info);
            }
        }

        printf("\r[Seq: %u | Polls: %u] "
               "Status: %s | "
               "P=%u.%u bar  F=%u.%u L/min  T=%u.%u C | "
               "Valve: %s (%02X) | "
               "CRC Errs: %u | "
               "Time: %s   ",
               local_copy.seq_count,
               local_copy.poll_count,
               local_copy.device_online ? "\033[32mONLINE\033[0m" : "\033[31mOFFLINE\033[0m",
               local_copy.registers[0] / 10, local_copy.registers[0] % 10,
               local_copy.registers[1] / 10, local_copy.registers[1] % 10,
               local_copy.registers[2] / 10, local_copy.registers[2] % 10,
               (local_copy.registers[3] == 1) ? "OPEN" : "CLOSED",
               local_copy.coil_state,
               local_copy.crc_errors,
               time_str);
        
        fflush(stdout);
        sleep(1);
    }

    munmap((void *)db_ptr, sizeof(ProcessDB));
    return EXIT_SUCCESS;
}
