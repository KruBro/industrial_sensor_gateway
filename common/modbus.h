#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define MB_EXCEPTION_FLAG        0x80
#define MB_EX_ILLEGAL_FUNCTION   0x01
#define MB_EX_ILLEGAL_ADDRESS    0x02
#define MB_EX_ILLEGAL_VALUE      0x03

#define MB_FC_READ_HOLDING       0x03
#define MB_FC_WRITE_COIL         0x05

#define MB_OK                    0
#define MB_ERR_SHORT            (-1)
#define MB_ERR_CRC              (-2)
#define MB_ERR_FUNCTION         (-3)
#define MB_ERR_LENGTH           (-4)

typedef struct {
    uint8_t  slave_addr;
    uint8_t  function_code;
    uint16_t start_reg;
    uint16_t quantity;
    uint8_t  data[252];
    uint16_t crc16;
} ModbusFrame;

uint16_t modbus_build_read_request(uint8_t addr, uint16_t start,uint16_t qty, uint8_t *out);
uint16_t modbus_build_write_coil_request(uint8_t addr, uint16_t coil, uint8_t on, uint8_t *out);
uint16_t modbus_build_read_response(uint8_t addr, const uint16_t *regs, uint16_t count, uint8_t *out);
uint16_t modbus_build_exception(uint8_t addr, uint8_t function, uint8_t exception_code, uint8_t *out);
int modbus_parse_request(const uint8_t *rx, uint16_t len, ModbusFrame *out);
int modbus_validate_frame(const uint8_t *buf, uint16_t len);

#endif
