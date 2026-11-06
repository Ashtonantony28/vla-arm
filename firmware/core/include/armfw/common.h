/* armfw common helpers: fixed-size types, little-endian access, small math.
 * Portable C11, no libm, no dynamic allocation. */
#ifndef ARMFW_COMMON_H
#define ARMFW_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ARMFW_N_JOINTS 6

#define ARMFW_FW_VERSION 0x0100u /* major<<8 | minor  -> 1.0 */

static inline void armfw_put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}
static inline void armfw_put_i16(uint8_t *p, int16_t v) { armfw_put_u16(p, (uint16_t)v); }
static inline void armfw_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}
static inline void armfw_put_i32(uint8_t *p, int32_t v) { armfw_put_u32(p, (uint32_t)v); }
static inline uint16_t armfw_get_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t armfw_get_i16(const uint8_t *p) { return (int16_t)armfw_get_u16(p); }
static inline uint32_t armfw_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t armfw_get_i32(const uint8_t *p) { return (int32_t)armfw_get_u32(p); }

static inline float armfw_fabsf(float x) { return x < 0.0f ? -x : x; }
static inline float armfw_fminf(float a, float b) { return a < b ? a : b; }
static inline float armfw_fmaxf(float a, float b) { return a > b ? a : b; }
static inline float armfw_clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline int32_t armfw_clampi(int32_t x, int32_t lo, int32_t hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
/* Round to nearest, ties away from zero, saturating to int16. */
static inline int16_t armfw_f2i16(float x) {
    if (x >= 32767.0f) return 32767;
    if (x <= -32768.0f) return -32768;
    return (int16_t)(x >= 0.0f ? (int32_t)(x + 0.5f) : -(int32_t)(-x + 0.5f));
}

#endif
