#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include "modbus.h"
#include "serial_io.h"
#include "master.h"

#ifndef SIM_BYTE_DELAY_US
#define SIM_BYTE_DELAY_US     0U
#endif



const char *mbm_strerror(mbm_status_t s)
{
    switch (s)
    {
    case MBM_OK:            return "OK";
    case MBM_ERR_ARGS:      return "bad arguments";
    case MBM_ERR_IO:        return "serial I/O error";
    case MBM_ERR_TIMEOUT:   return "no reply (timeout)";
    case MBM_ERR_SHORT:     return "reply incomplete";
    case MBM_ERR_CRC:       return "bad CRC";
    case MBM_ERR_ADDRESS:   return "reply from wrong slave address";
    case MBM_ERR_FUNCTION:  return "unexpected function code in reply";
    case MBM_ERR_BYTECOUNT: return "byte count does not match request";
    case MBM_ERR_EXCEPTION: return "slave returned an exception";
    case MBM_INCOMPLETE:    return "transaction not completed";
    }
    return "unknown error";
}

const char *exception_name(uint8_t code)
{
    switch (code)
    {
    case MB_EX_ILLEGAL_FUNCTION: return "illegal function";
    case MB_EX_ILLEGAL_ADDRESS:  return "illegal data address";
    case MB_EX_ILLEGAL_VALUE:    return "illegal data value";
    default:                     return "unknown";
    }
}

static int remaining_ms(const struct timespec *start, int total_ms)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long long used_ms =
        ((long long)(now.tv_sec - start->tv_sec) * 1000000000LL +
         (now.tv_nsec - start->tv_nsec)) / 1000000LL;

    long long left = total_ms - used_ms;
    return (left > 0) ? (int)left : 0;
}

mbm_status_t modbus_read_registers(int fd, uint8_t addr, uint16_t start,
                                   uint16_t count, uint16_t *out, uint8_t *exc_code)
{
    uint8_t request[16];
    uint8_t response[MAX_FRAME_LEN];
    struct timespec t0;

    if (fd < 0 || out == NULL || count < 1 || count > 125)
    {
        return MBM_ERR_ARGS;
    }

    clock_gettime(CLOCK_MONOTONIC, &t0);

    tcflush(fd, TCIFLUSH);

    uint16_t req_len = modbus_build_read_request(addr, start, count, request);

    if (send_serial_frame(fd, request, req_len, SIM_BYTE_DELAY_US) < 0)
    {
        return MBM_ERR_IO;
    }

    ssize_t got = read_serial_frame(fd, response, MIN_REPLY_LEN,
                                    remaining_ms(&t0, RESPONSE_TIMEOUT_MS));
    if (got < 0)
    {
        return MBM_ERR_IO;
    }
    if (got == 0)
    {
        return MBM_ERR_TIMEOUT;
    }
    if (got < MIN_REPLY_LEN)
    {
        return MBM_ERR_SHORT;
    }

    if (response[1] & MB_EXCEPTION_FLAG)
    {
        if (modbus_validate_frame(response, MIN_REPLY_LEN) != MB_OK)
        {
            return MBM_ERR_CRC;
        }
        if (response[0] != addr)
        {
            return MBM_ERR_ADDRESS;
        }
        if ((response[1] & ~MB_EXCEPTION_FLAG) != MB_FC_READ_HOLDING)
        {
            return MBM_ERR_FUNCTION;
        }

        if (exc_code != NULL)
        {
            *exc_code = response[2];
        }
        return MBM_ERR_EXCEPTION;
    }

    if (response[2] != 2 * count)
    {
        return MBM_ERR_BYTECOUNT;
    }

    ssize_t total_len = MIN_REPLY_LEN + 2 * count;
    ssize_t need = total_len - MIN_REPLY_LEN;

    got = read_serial_frame(fd, response + MIN_REPLY_LEN, need,
                            remaining_ms(&t0, RESPONSE_TIMEOUT_MS));
    if (got < 0)
    {
        return MBM_ERR_IO;
    }
    if (got < need)
    {
        return MBM_ERR_SHORT;
    }

    if (modbus_validate_frame(response, (uint16_t)total_len) != MB_OK)
    {
        return MBM_ERR_CRC;
    }
    if (response[0] != addr)
    {
        return MBM_ERR_ADDRESS;
    }
    if (response[1] != MB_FC_READ_HOLDING)
    {
        return MBM_ERR_FUNCTION;
    }

    for (uint16_t j = 0; j < count; j++)
    {
        uint8_t hi = response[3 + 2 * j];
        uint8_t lo = response[4 + 2 * j];
        out[j] = (uint16_t)((hi << 8) | lo);
    }

    return MBM_OK;
}

static void sleep_ms(unsigned ms)
{
    usleep(ms * 1000U);
}

int is_retryable(mbm_status_t st)
{
    switch (st)
    {
    case MBM_OK:
    case MBM_ERR_EXCEPTION:
    case MBM_ERR_IO:
    case MBM_ERR_ARGS:
        return 0;

    /* Lost or damaged replies, and stray frames on a single-slave bus. */
    case MBM_ERR_TIMEOUT:
    case MBM_ERR_SHORT:
    case MBM_ERR_CRC:
    case MBM_ERR_ADDRESS:
    case MBM_ERR_FUNCTION:
    case MBM_ERR_BYTECOUNT:
        return 1;

    default:
        return 0;
    }
}

mbm_status_t modbus_read_registers_retry(int fd, uint8_t addr, uint16_t start,
                                         uint16_t count, uint16_t *out, uint8_t *exc_code,
                                         int max_attempts, int *attempts_used)
{
    int unused;
    mbm_status_t ret = MBM_ERR_ARGS;

    if (attempts_used == NULL)
    {
        attempts_used = &unused;
    }
    *attempts_used = 0;

    for (int i = 1; i <= max_attempts; i++)
    {
        *attempts_used = i;
        ret = modbus_read_registers(fd, addr, start, count, out, exc_code);
        if (!is_retryable(ret))
        {
            return ret;
        }

        fprintf(stderr, "attempt %d/%d: %s\n", i, max_attempts, mbm_strerror(ret));

        if (i < max_attempts)
        {
            sleep_ms(RETRY_DELAY_MS);
        }
    }

    return ret;
}

mbm_status_t modbus_send_request_async(int fd, uint8_t addr, uint16_t start, uint16_t count)
{
    uint8_t request[16];
    if (fd < 0 || count < 1 || count > 125) {
        return MBM_ERR_ARGS;
    }
    tcflush(fd, TCIFLUSH);
    uint16_t req_len = modbus_build_read_request(addr, start, count, request);
    if (send_serial_frame(fd, request, req_len, SIM_BYTE_DELAY_US) < 0) {
        return MBM_ERR_IO;
    }
    return MBM_OK;
}

mbm_status_t modbus_send_write_coil_async(int fd, uint8_t addr, uint16_t coil, uint8_t on)
{
    uint8_t request[16];
    if(fd < 0)
        return MBM_ERR_ARGS;
    
    tcflush(fd, TCIFLUSH);
    uint16_t req_len = modbus_build_write_coil_request(addr, coil, on, request);
    if(send_serial_frame(fd, request, req_len, SIM_BYTE_DELAY_US) < 0)
        return MBM_ERR_IO;
    
    return MBM_OK;
}

mbm_status_t modbus_parse_buffer_async(const uint8_t *buf, size_t buf_len, uint16_t count, 
                                       uint8_t addr, uint16_t *out, uint8_t *exc_code, uint8_t expected_fc)
{
    // A frame must be at least 5 bytes (MIN_REPLY_LEN) before we can evaluate it
    if (buf_len < MIN_REPLY_LEN) {
        return MBM_INCOMPLETE;
    }

    // 1. CHECK FOR SLAVE EXCEPTIONS (5-byte frames)
    if (buf[1] & MB_EXCEPTION_FLAG) {
        if (modbus_validate_frame((uint8_t *)buf, MIN_REPLY_LEN) != MB_OK) {
            return MBM_ERR_CRC;
        }
        if (buf[0] != addr) {
            return MBM_ERR_ADDRESS;
        }
        if ((buf[1] & ~MB_EXCEPTION_FLAG) != expected_fc) {
            return MBM_ERR_FUNCTION;
        }
        if (exc_code != NULL) {
            *exc_code = buf[2];
        }
        return MBM_ERR_EXCEPTION;
    }

    // 2. PARSE READ HOLDING REGISTERS RESPONSE (FC 0x03)
    if (expected_fc == MB_FC_READ_HOLDING) {
        if (buf[2] != 2 * count) {
            return MBM_ERR_BYTECOUNT;
        }

        size_t total_expected_len = MIN_REPLY_LEN + (2 * count);
        if (buf_len < total_expected_len) {
            return MBM_INCOMPLETE;
        }

        if (modbus_validate_frame((uint8_t *)buf, (uint16_t)total_expected_len) != MB_OK) {
            return MBM_ERR_CRC;
        }
        if (buf[0] != addr) {
            return MBM_ERR_ADDRESS;
        }
        if (buf[1] != MB_FC_READ_HOLDING) {
            return MBM_ERR_FUNCTION;
        }

        for (uint16_t j = 0; j < count; j++) {
            uint8_t hi = buf[3 + 2 * j];
            uint8_t lo = buf[4 + 2 * j];
            out[j] = (uint16_t)((hi << 8) | lo);
        }
        return MBM_OK;
    }

    // 3. PARSE WRITE SINGLE COIL RESPONSE ECHO (FC 0x05)
    else if (expected_fc == MB_FC_WRITE_COIL) {
        // A standard write coil reply is an exact 8-byte echo of the request
        size_t total_expected_len = 8;
        if (buf_len < total_expected_len) {
            return MBM_INCOMPLETE;
        }

        if (modbus_validate_frame((uint8_t *)buf, (uint16_t)total_expected_len) != MB_OK) {
            return MBM_ERR_CRC;
        }
        if (buf[0] != addr) {
            return MBM_ERR_ADDRESS;
        }
        if (buf[1] != MB_FC_WRITE_COIL) {
            return MBM_ERR_FUNCTION;
        }

        // Note: The loop level caller will verify if the 8-byte response array matches 
        // the sent array byte-for-byte to confirm it's a valid echo.
        return MBM_OK;
    }

    return MBM_ERR_ARGS;
}

