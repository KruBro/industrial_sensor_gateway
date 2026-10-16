#ifndef CRC16_H
#define CRC16_H

#include <stdio.h>
#include <stdint.h>

uint16_t modbus_crc16(const uint8_t *buff, uint16_t len);

#endif