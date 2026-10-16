#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include "db.h"

ProcessDB *db_init(const char *db_path)
{
    int fd = open(db_path, O_RDWR | O_CREAT, 0666);
    if(fd < 0)
    {
        perror("open");
        return NULL;
    }

    if(ftruncate(fd, sizeof(ProcessDB)) < 0)
    {
        perror("ftruncate");
        close(fd);
        return NULL;
    }

    ProcessDB *db = (ProcessDB *)mmap(NULL, sizeof(ProcessDB), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    close(fd);

    if(db == MAP_FAILED)
    {
        perror("mmap");
        return NULL;
    }

    memset(db, 0, sizeof(ProcessDB));
    db->device_online = 1;
    return db;
}

void db_update_success(ProcessDB *db, const uint16_t *values, uint16_t count)
{
    if(!db)
        return;
    
    db->seq_count++;

    for(uint16_t i = 0; i < count; i++)
    {
        db->registers[i] = values[i];
    }

    if(count == 4)
    {
        db->coil_state = (uint8_t)values[3];
    }

    db->device_online = 1;
    db->failed_polls = 0;
    db->last_poll_time = time(NULL);
    db->poll_count++;

    db->seq_count++;
}

void db_update_failure(ProcessDB *db, int is_crc_error)
{
    if(!db)
        return;
    
    db->seq_count++;

    if(is_crc_error)
    {
        db->crc_errors++;
    }
    else
    {
        db->failed_polls++;
        if(db->failed_polls >= 3)
        {
            db->device_online = 0;
        }
    }

    db->seq_count++;
}

void db_close(ProcessDB *db)
{
    if(!db)
        return;
    
    db->device_online = 0;
    if(munmap(db, sizeof(ProcessDB)) < 0)
    {
        perror("munmap");
    }
}
