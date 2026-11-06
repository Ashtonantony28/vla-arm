/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0. Check("123456789") = 0x29B1. */
#ifndef ARMFW_CRC16_H
#define ARMFW_CRC16_H

#include "armfw/common.h"

#define CRC16_INIT 0xFFFFu

uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t len);
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

#endif
