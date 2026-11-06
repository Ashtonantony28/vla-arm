/* libarmfw - stable C API over the firmware core, for host tools (e.g. the Python driver via
 * ctypes) to cross-test framing, CRC and payload layouts against the exact firmware code, and to
 * run the whole firmware in-process against simulated servos.
 *
 * Stability rules: names and signatures below are frozen for ARMFW_API_VERSION 1. New functions
 * may be added; existing ones are never changed. Every function is plain C with fixed-width
 * integer arguments; there is no allocation inside the library: objects (parser, sim) live in
 * caller-provided memory of armfw_*_sizeof() bytes, aligned to 16 bytes.
 * Return conventions: sizes are 0 on error; int results are 1 = ok, 0 = failure.
 */
#ifndef ARMFW_API_H
#define ARMFW_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ARMFW_BUILDING_LIB)
#define ARMFW_API __attribute__((visibility("default")))
#else
#define ARMFW_API
#endif

#define ARMFW_API_VERSION 1u

/* ---- version / CRC / framing ---- */
ARMFW_API uint32_t armfw_api_version(void);
ARMFW_API uint16_t armfw_fw_version(void);
ARMFW_API uint16_t armfw_crc16(const uint8_t *data, size_t len);
/* Full frame (sync..crc) into out. Returns frame length (len + 8) or 0. */
ARMFW_API size_t armfw_encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t len,
                                    uint8_t *out, size_t out_cap);
/* Expected payload length of a message TYPE, -1 if unknown. */
ARMFW_API int armfw_expected_len(uint8_t type);

/* ---- streaming parser ---- */
typedef struct {
    uint8_t ver;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[240];
} armfw_frame_t; /* 244 bytes, no padding */

ARMFW_API size_t armfw_parser_sizeof(void);
ARMFW_API void armfw_parser_init(void *parser);
/* Feed up to len bytes. Stops early once max_frames frames are written to out. *consumed (may be
 * NULL) receives the number of input bytes used; call again with the rest. Returns frames written. */
ARMFW_API size_t armfw_parser_feed(void *parser, const uint8_t *data, size_t len, armfw_frame_t *out,
                                   size_t max_frames, size_t *consumed);
/* out[6] = frames_ok, crc_errors, ver_errors, len_errors, bytes_skipped, overflows */
ARMFW_API void armfw_parser_stats(const void *parser, uint32_t out[6]);

/* ---- payload pack (returns payload length) / unpack (returns 1 ok, 0 bad length) ---- */
ARMFW_API size_t armfw_pack_set_targets(uint16_t move_ms, const int16_t pos[6], uint8_t flags, uint8_t *out);
ARMFW_API int armfw_unpack_set_targets(const uint8_t *p, size_t len, uint16_t *move_ms, int16_t pos[6],
                                       uint8_t *flags);
ARMFW_API size_t armfw_pack_config(uint8_t joint, uint8_t key, int32_t value, uint8_t *out);
ARMFW_API int armfw_unpack_config(const uint8_t *p, size_t len, uint8_t *joint, uint8_t *key, int32_t *value);
ARMFW_API size_t armfw_pack_clear_fault(uint16_t mask, uint8_t *out);
ARMFW_API int armfw_unpack_clear_fault(const uint8_t *p, size_t len, uint16_t *mask);
ARMFW_API size_t armfw_pack_set_stream(uint16_t period_us, uint8_t *out);
ARMFW_API int armfw_unpack_set_stream(const uint8_t *p, size_t len, uint16_t *period_us);
ARMFW_API size_t armfw_pack_pong(uint16_t fw_version, uint32_t uptime_ms, uint8_t proto_ver, uint8_t *out);
ARMFW_API int armfw_unpack_pong(const uint8_t *p, size_t len, uint16_t *fw_version, uint32_t *uptime_ms,
                                uint8_t *proto_ver);
ARMFW_API size_t armfw_pack_ack(uint8_t seq, uint8_t type, uint8_t *out);
ARMFW_API int armfw_unpack_ack(const uint8_t *p, size_t len, uint8_t *seq, uint8_t *type);
ARMFW_API size_t armfw_pack_nack(uint8_t seq, uint8_t type, uint8_t error, uint8_t *out);
ARMFW_API int armfw_unpack_nack(const uint8_t *p, size_t len, uint8_t *seq, uint8_t *type, uint8_t *error);

/* STATE in a flat, naturally aligned layout (68 bytes incl. 2 tail padding bytes):
 * off 0 t_us u32 | 4 fault_flags u16 | 6 mode u8 | 7 last_host_seq u8 | 8 pos_mrad i16[6] |
 * 20 vel_mrad_s i16[6] | 32 current_ma i16[6] | 44 temp_c u8[6] | 50 joint_status u8[6] |
 * 56 bus_mv u16 | 58 bus_ma i16 | 60 loop_jitter_max_us u16 | 62 loop_overruns u16 | 64 cmd_pos_crc u16 */
typedef struct {
    uint32_t t_us;
    uint16_t fault_flags;
    uint8_t mode;
    uint8_t last_host_seq;
    int16_t pos_mrad[6];
    int16_t vel_mrad_s[6];
    int16_t current_ma[6];
    uint8_t temp_c[6];
    uint8_t joint_status[6];
    uint16_t bus_mv;
    int16_t bus_ma;
    uint16_t loop_jitter_max_us;
    uint16_t loop_overruns;
    uint16_t cmd_pos_crc;
} armfw_state_t;

ARMFW_API size_t armfw_state_sizeof(void);
ARMFW_API size_t armfw_pack_state(const armfw_state_t *st, uint8_t *out);
ARMFW_API int armfw_unpack_state(const uint8_t *p, size_t len, armfw_state_t *st);

/* ---- in-process firmware simulation (control core + fake HAL + 6 fake servos) ---- */
ARMFW_API size_t armfw_sim_sizeof(void);
ARMFW_API void armfw_sim_init(void *sim, uint32_t start_us);
/* Run n control periods (1 ms each) of virtual time. */
ARMFW_API void armfw_sim_step(void *sim, uint32_t n);
ARMFW_API uint32_t armfw_sim_time_us(const void *sim);
/* Host -> board bytes (returns accepted), board -> host bytes (returns copied). */
ARMFW_API size_t armfw_sim_host_write(void *sim, const uint8_t *data, size_t len);
ARMFW_API size_t armfw_sim_host_read(void *sim, uint8_t *buf, size_t cap);
/* Environment and fault injection. joint is 0..5. */
ARMFW_API void armfw_sim_set_estop_closed(void *sim, int closed);
ARMFW_API void armfw_sim_set_supply_mv(void *sim, uint32_t mv);
ARMFW_API void armfw_sim_set_servo_silent(void *sim, int joint, int silent);
ARMFW_API void armfw_sim_set_servo_temp(void *sim, int joint, int temp_c);
ARMFW_API void armfw_sim_inject_current(void *sim, int joint, int enable, int current_ma);
ARMFW_API void armfw_sim_set_echo(void *sim, int echo);
/* Introspection: board mode, fault flags, true (simulated) servo position in mrad. */
ARMFW_API int armfw_sim_mode(const void *sim);
ARMFW_API uint16_t armfw_sim_faults(const void *sim);
ARMFW_API float armfw_sim_servo_pos_mrad(const void *sim, int joint);

#ifdef __cplusplus
}
#endif

#endif
