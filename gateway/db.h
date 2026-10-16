#ifndef DB_H
#define DB_H

#include <stdint.h>
#include <time.h>
#include <assert.h>

typedef struct {
    time_t   last_poll_time;   // 8 bytes (64-bit Unix timestamp)
    uint32_t seq_count;        // 4 bytes
    uint32_t poll_count;       // 4 bytes
    uint32_t crc_errors;       // 4 bytes
    uint32_t failed_polls;     // 4 bytes
    uint16_t registers[4];     // 8 bytes (Pressure, Flow, Temp, Valve)
    uint8_t  coil_state;       // 1 byte
    uint8_t  device_online;    // 1 byte
    uint8_t  padding[6];      // 10 bytes (Explicit padding to cleanly hit 44 bytes)
} ProcessDB;

static_assert(sizeof(ProcessDB) == 40, "ProcessDB struct size must be exactly 40 bytes!");

ProcessDB *db_init(const char *db_path);
void db_update_success(ProcessDB *db, const uint16_t *values, uint16_t count);
void db_update_failure(ProcessDB *db, int is_crc_error);
void db_close(ProcessDB *db);

#endif
