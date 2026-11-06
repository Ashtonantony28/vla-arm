/* Host <-> board framed protocol, version 1 (spec/protocol.md).
 *
 * Frame: A5 5A | VER | TYPE | SEQ | LEN | PAYLOAD[LEN] | CRC16 LE (over VER..PAYLOAD)
 */
#ifndef ARMFW_PROTO_H
#define ARMFW_PROTO_H

#include "armfw/common.h"

#define PROTO_SYNC0 0xA5u
#define PROTO_SYNC1 0x5Au
#define PROTO_VER 0x01u
#define PROTO_MAX_PAYLOAD 240u
#define PROTO_HEADER_LEN 6u /* sync0 sync1 ver type seq len */
#define PROTO_OVERHEAD 8u   /* header + crc */
#define PROTO_MAX_FRAME (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)

/* Message types */
enum {
    PROTO_T_PING = 0x01,
    PROTO_T_SET_TARGETS = 0x02,
    PROTO_T_CONFIG = 0x03,
    PROTO_T_ESTOP = 0x04,
    PROTO_T_CLEAR_FAULT = 0x05,
    PROTO_T_SET_STREAM = 0x06,
    PROTO_T_PONG = 0x81,
    PROTO_T_STATE = 0x82,
    PROTO_T_ACK = 0x83,
    PROTO_T_NACK = 0x8F,
};

/* Payload sizes */
enum {
    PROTO_LEN_PING = 0,
    PROTO_LEN_SET_TARGETS = 15,
    PROTO_LEN_CONFIG = 6,
    PROTO_LEN_ESTOP = 0,
    PROTO_LEN_CLEAR_FAULT = 2,
    PROTO_LEN_SET_STREAM = 2,
    PROTO_LEN_PONG = 7,
    PROTO_LEN_STATE = 66,
    PROTO_LEN_ACK = 2,
    PROTO_LEN_NACK = 3,
};

/* NACK error codes */
enum {
    PROTO_ERR_BAD_LEN = 1,
    PROTO_ERR_BAD_VALUE = 2,
    PROTO_ERR_UNKNOWN_TYPE = 3,
    PROTO_ERR_FAULTED = 4,
    PROTO_ERR_UNKNOWN_KEY = 5,
};

/* SET_TARGETS flags */
#define PROTO_FLAG_TORQUE 0x01u

/* CONFIG keys */
enum {
    PROTO_CFG_OFFSET_TICKS = 0x01,
    PROTO_CFG_DIRECTION = 0x02,
    PROTO_CFG_LIMIT_MIN = 0x03,
    PROTO_CFG_LIMIT_MAX = 0x04,
    PROTO_CFG_VMAX = 0x05,
    PROTO_CFG_CURRENT_LIMIT = 0x06,
    PROTO_CFG_WD_HOLD_MS = 0x10,
    PROTO_CFG_WD_OFF_MS = 0x11,
};
#define PROTO_CFG_JOINT_GLOBAL 0xFFu

/* Modes */
enum { PROTO_MODE_IDLE = 0, PROTO_MODE_ACTIVE = 1, PROTO_MODE_HOLD = 2, PROTO_MODE_FAULT = 3 };

/* Fault flags */
#define FAULT_ESTOP_SOFT (1u << 0)
#define FAULT_ESTOP_HW (1u << 1)
#define FAULT_WATCHDOG (1u << 2)
#define FAULT_OVERCURRENT (1u << 3)
#define FAULT_OVERTEMP (1u << 4)
#define FAULT_SERVO_COMMS (1u << 5)
#define FAULT_BUS_UNDERVOLT (1u << 6)
#define FAULT_LIMIT_CLAMP (1u << 7)
#define FAULT_DEFINED_MASK 0x00FFu

/* ---- payload structs ---- */
typedef struct {
    uint16_t move_ms;
    int16_t pos[ARMFW_N_JOINTS];
    uint8_t flags;
} proto_set_targets_t;

typedef struct {
    uint8_t joint;
    uint8_t key;
    int32_t value;
} proto_config_t;

typedef struct {
    uint16_t mask;
} proto_clear_fault_t;

typedef struct {
    uint16_t period_us;
} proto_set_stream_t;

typedef struct {
    uint16_t fw_version;
    uint32_t uptime_ms;
    uint8_t proto_ver;
} proto_pong_t;

typedef struct {
    int16_t pos_mrad;
    int16_t vel_mrad_s;
    int16_t current_ma;
    uint8_t temp_c;
    uint8_t status;
} proto_joint_state_t;

typedef struct {
    uint32_t t_us;
    uint16_t fault_flags;
    uint8_t mode;
    uint8_t last_host_seq;
    proto_joint_state_t joint[ARMFW_N_JOINTS];
    uint16_t bus_mv;
    int16_t bus_ma;
    uint16_t loop_jitter_max_us;
    uint16_t loop_overruns;
    uint16_t cmd_pos_crc;
} proto_state_t;

typedef struct {
    uint8_t seq;
    uint8_t type;
} proto_ack_t;

typedef struct {
    uint8_t seq;
    uint8_t type;
    uint8_t error;
} proto_nack_t;

typedef struct {
    uint8_t ver;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/* Expected payload length for a known TYPE, or -1 if TYPE is unknown. */
int proto_expected_len(uint8_t type);
/* True for TYPE values that the host may send to the board. */
bool proto_is_host_type(uint8_t type);

/* Encode a full frame into out. Returns bytes written (len + 8), or 0 if cap too small / len > 240. */
size_t proto_encode(uint8_t *out, size_t cap, uint8_t type, uint8_t seq, const uint8_t *payload,
                    size_t len);

/* ---- pack/unpack: pack returns payload length written; unpack returns false on length mismatch ---- */
size_t proto_pack_set_targets(const proto_set_targets_t *m, uint8_t *out);
bool proto_unpack_set_targets(const uint8_t *p, size_t len, proto_set_targets_t *m);
size_t proto_pack_config(const proto_config_t *m, uint8_t *out);
bool proto_unpack_config(const uint8_t *p, size_t len, proto_config_t *m);
size_t proto_pack_clear_fault(const proto_clear_fault_t *m, uint8_t *out);
bool proto_unpack_clear_fault(const uint8_t *p, size_t len, proto_clear_fault_t *m);
size_t proto_pack_set_stream(const proto_set_stream_t *m, uint8_t *out);
bool proto_unpack_set_stream(const uint8_t *p, size_t len, proto_set_stream_t *m);
size_t proto_pack_pong(const proto_pong_t *m, uint8_t *out);
bool proto_unpack_pong(const uint8_t *p, size_t len, proto_pong_t *m);
size_t proto_pack_state(const proto_state_t *m, uint8_t *out);
bool proto_unpack_state(const uint8_t *p, size_t len, proto_state_t *m);
size_t proto_pack_ack(const proto_ack_t *m, uint8_t *out);
bool proto_unpack_ack(const uint8_t *p, size_t len, proto_ack_t *m);
size_t proto_pack_nack(const proto_nack_t *m, uint8_t *out);
bool proto_unpack_nack(const uint8_t *p, size_t len, proto_nack_t *m);

/* ---- streaming parser with resynchronisation ----
 * Usage: for each received byte: proto_parser_feed(p, b); while (proto_parser_poll(p, &f)) handle(f);
 * The parser keeps at most one maximum-size frame of bytes. When a candidate frame fails
 * (bad VER, LEN > 240, bad CRC) only its first byte is discarded and scanning restarts from the
 * next byte, so a real frame hidden inside a corrupt one is still found. */
typedef struct {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t ver_errors;
    uint32_t len_errors;
    uint32_t bytes_skipped;
    uint32_t overflows;
} proto_parser_stats_t;

typedef struct {
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t len;
    proto_parser_stats_t stats;
} proto_parser_t;

void proto_parser_init(proto_parser_t *p);
/* Append one byte. Returns false (byte dropped, overflow counted) if the buffer is full, which only
 * happens when the caller does not poll after each feed. */
bool proto_parser_feed(proto_parser_t *p, uint8_t b);
/* Extract the next complete valid frame, if any. */
bool proto_parser_poll(proto_parser_t *p, proto_frame_t *out);

typedef void (*proto_frame_cb)(void *ctx, const proto_frame_t *f);
/* Convenience: feed n bytes, polling after each and calling cb for each frame. Returns frame count. */
size_t proto_parser_push(proto_parser_t *p, const uint8_t *data, size_t n, proto_frame_cb cb,
                         void *ctx);

#endif
