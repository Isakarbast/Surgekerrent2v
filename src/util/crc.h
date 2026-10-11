#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t seed);
uint32_t crc32(const uint8_t *data, size_t len);

#endif
