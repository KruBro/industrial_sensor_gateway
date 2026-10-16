
#include "crc16.h"

uint16_t modbus_crc16(const uint8_t *buff, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	const uint16_t pol = 0xA001;


	for(uint16_t i = 0; i < len; i++)
	{
		crc = crc ^ buff[i];
		for(uint8_t j = 0; j < 8; j++)
		{
			if(crc & 1U)
			{
				crc >>= 1U;
				crc ^= pol;
			}
			else
			{
				crc >>= 1U;
			}
		}
	}
	return crc;
}

