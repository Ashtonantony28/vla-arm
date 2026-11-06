#include "armfw/crc16.h"

/* Nibble table: 32 bytes of flash, two lookups per byte. */
static const uint16_t k_crc_nibble[16] = {
    0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
    0x8108u, 0x9129u, 0xA14Au, 0xB16Bu, 0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
};

uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        crc = (uint16_t)((crc << 4) ^ k_crc_nibble[((crc >> 12) ^ (b >> 4)) & 0x0Fu]);
        crc = (uint16_t)((crc << 4) ^ k_crc_nibble[((crc >> 12) ^ (b & 0x0Fu)) & 0x0Fu]);
    }
    return crc;
}

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    return crc16_update(CRC16_INIT, data, len);
}
