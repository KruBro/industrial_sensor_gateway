#ifndef POLLER_H
#define POLLER_H

#include <stdint.h>
#include "master.h"
#define POLL_INTERVAL_MS      100

typedef enum {
    STATE_IDLE,
    STATE_WAIT_REPLY,
    STATE_RETRY_PAUSE
} trans_state_t;

struct modbus_transaction {
    trans_state_t state;
    uint8_t addr;
    uint16_t count;
    int attempt;
    struct timespec deadline;
    uint8_t rx_buf[MAX_FRAME_LEN];
    size_t rx_len;
    uint8_t expected_fc;
    uint8_t tx_frame[8];
    uint16_t actual_write_value;
};

struct pending_request {
    int has_request;
    uint8_t addr;
    uint16_t count;

    uint8_t is_write_coil;
    uint16_t write_value;
};



int poller_run(int fd, uint8_t addr, uint16_t count, unsigned interval_ms, uint16_t valve_state, const char* log_path);

#endif