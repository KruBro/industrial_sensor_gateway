#ifndef MASTER_H
#define MASTER_H

#include <stdint.h>

#define START_REG       0
#define MAX_ATTEMPTS    3

#define RESPONSE_TIMEOUT_MS   2000
#define MIN_REPLY_LEN         5
#define MAX_FRAME_LEN         256
#define RETRY_DELAY_MS        100

typedef enum {
    MBM_OK = 0,
    MBM_ERR_ARGS,
    MBM_ERR_IO,
    MBM_ERR_TIMEOUT,
    MBM_ERR_SHORT,
    MBM_ERR_CRC,
    MBM_ERR_ADDRESS,
    MBM_ERR_FUNCTION,
    MBM_ERR_BYTECOUNT,
    MBM_ERR_EXCEPTION,
    MBM_INCOMPLETE
} mbm_status_t;

#define MBM_STATUS_COUNT    (MBM_ERR_EXCEPTION + 1)

/* Text for a status code, and for a Modbus exception code. */
const char *mbm_strerror(mbm_status_t s);
const char *exception_name(uint8_t code);

/* One read-holding-registers transaction: send, receive, validate, decode.
 * On MBM_ERR_EXCEPTION the slave's exception code is stored in *exc_code. */
mbm_status_t modbus_read_registers(int fd, uint8_t addr, uint16_t start,
                                   uint16_t count, uint16_t *out, uint8_t *exc_code);

/* The same, retried up to max_attempts times for lost or damaged replies.
 * *attempts_used (may be NULL) receives the number of attempts made. */
mbm_status_t modbus_read_registers_retry(int fd, uint8_t addr, uint16_t start,
                                         uint16_t count, uint16_t *out, uint8_t *exc_code,
                                         int max_attempts, int *attempts_used);

mbm_status_t modbus_send_request_async(int fd, uint8_t addr, uint16_t start, uint16_t count);

mbm_status_t modbus_parse_buffer_async(const uint8_t *buf, size_t buf_len, uint16_t count, 
                                       uint8_t addr, uint16_t *out, uint8_t *exc_code, uint8_t expected_fc);

mbm_status_t modbus_send_write_coil_async(int fd, uint8_t addr, uint16_t coil, uint8_t on);
int is_retryable(mbm_status_t st);

#endif
