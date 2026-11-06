/* Feetech STS/SCS-style serial bus servo protocol (Dynamixel-1.0 compatible framing).
 *
 * Instruction packet: FF FF ID LEN INSTR PARAM... CHK     LEN = nparams + 2
 * Status packet:      FF FF ID LEN ERR   PARAM... CHK     LEN = nparams + 2
 * CHK = ~(ID + LEN + INSTR/ERR + sum(PARAM)) & 0xFF
 *
 * STS3215 register map subset used here (RAM area):
 *   0x28 torque enable (1)   0x2A goal position (2)
 *   0x38 present position (2) 0x3A present speed (2, sign bit 15) 0x3C present load (2, sign bit 10)
 *   0x3E present voltage (1, 0.1 V) 0x3F present temperature (1, C) 0x40 async write flag (1)
 *   0x41 servo status (1)  0x42 moving (1)  0x43..0x44 (unused here)  0x45 present current (2, 6.5 mA/LSB, sign bit 15)
 * One READ of 15 bytes at 0x38 returns position..current in one transaction.
 */
#ifndef ARMFW_SERVO_BUS_H
#define ARMFW_SERVO_BUS_H

#include "armfw/common.h"

#define SB_HDR0 0xFFu
#define SB_HDR1 0xFFu
#define SB_BROADCAST_ID 0xFEu

enum {
    SB_INST_PING = 0x01,
    SB_INST_READ = 0x02,
    SB_INST_WRITE = 0x03,
    SB_INST_REG_WRITE = 0x04,
    SB_INST_ACTION = 0x05,
    SB_INST_RESET = 0x06,
    SB_INST_SYNC_READ = 0x82,
    SB_INST_SYNC_WRITE = 0x83,
};

enum {
    SB_REG_TORQUE_ENABLE = 0x28,
    SB_REG_GOAL_POSITION = 0x2A,
    SB_REG_PRESENT_POSITION = 0x38,
    SB_REG_PRESENT_SPEED = 0x3A,
    SB_REG_PRESENT_LOAD = 0x3C,
    SB_REG_PRESENT_VOLTAGE = 0x3E,
    SB_REG_PRESENT_TEMPERATURE = 0x3F,
    SB_REG_STATUS = 0x41,
    SB_REG_MOVING = 0x42,
    SB_REG_PRESENT_CURRENT = 0x45,
};

#define SB_FEEDBACK_ADDR SB_REG_PRESENT_POSITION
#define SB_FEEDBACK_LEN 15u /* 0x38 .. 0x46 */
#define SB_CURRENT_MA_PER_LSB 6.5f
#define SB_MAX_PARAMS 64u
#define SB_MAX_PACKET (SB_MAX_PARAMS + 6u)

/* Bus timing at 1 Mbps, 8N1: 10 bit times per byte. */
#define SB_BAUD 1000000u
#define SB_US_PER_BYTE_X10 100u /* 10.0 us per byte, x10 for integer math */

uint8_t sb_checksum(const uint8_t *from_id, size_t n); /* n bytes starting at ID, excluding CHK */

/* Generic instruction packet. Returns bytes written or 0 if it does not fit. */
size_t sb_encode(uint8_t *out, size_t cap, uint8_t id, uint8_t instr, const uint8_t *params,
                 size_t nparams);
size_t sb_encode_ping(uint8_t *out, size_t cap, uint8_t id);
size_t sb_encode_read(uint8_t *out, size_t cap, uint8_t id, uint8_t addr, uint8_t len);
size_t sb_encode_write(uint8_t *out, size_t cap, uint8_t id, uint8_t addr, const uint8_t *data,
                       size_t n);
/* SYNC_WRITE: data holds n_ids blocks of data_len bytes, one per id. */
size_t sb_encode_sync_write(uint8_t *out, size_t cap, uint8_t addr, uint8_t data_len,
                            const uint8_t *ids, const uint8_t *data, size_t n_ids);
/* SYNC_WRITE of 2-byte goal positions (ticks, little-endian) to n servos. */
size_t sb_encode_sync_goal(uint8_t *out, size_t cap, const uint8_t *ids, const uint16_t *ticks,
                           size_t n);
/* SYNC_WRITE of the torque-enable byte to n servos. */
size_t sb_encode_sync_torque(uint8_t *out, size_t cap, const uint8_t *ids, size_t n, bool enable);
/* Status packet (used by the fake servo). */
size_t sb_encode_status(uint8_t *out, size_t cap, uint8_t id, uint8_t error, const uint8_t *params,
                        size_t nparams);

/* Decoded packet (instruction or status share the layout: 'code' is INSTR or ERR). */
typedef struct {
    uint8_t id;
    uint8_t code;
    uint8_t nparams;
    uint8_t params[SB_MAX_PARAMS];
} sb_packet_t;

typedef struct {
    uint32_t packets_ok;
    uint32_t checksum_errors;
    uint32_t len_errors;
    uint32_t bytes_skipped;
    uint32_t overflows;
} sb_parser_stats_t;

/* Streaming packet parser with resync (same scheme as proto_parser_t). */
typedef struct {
    uint8_t buf[SB_MAX_PACKET];
    uint16_t len;
    sb_parser_stats_t stats;
} sb_parser_t;

void sb_parser_init(sb_parser_t *p);
bool sb_parser_feed(sb_parser_t *p, uint8_t b);
bool sb_parser_poll(sb_parser_t *p, sb_packet_t *out);

/* Feedback block read from SB_FEEDBACK_ADDR. */
typedef struct {
    uint16_t pos_ticks;
    int16_t speed_ticks_s;
    int16_t load_permille; /* +-1000 = +-100 % */
    uint8_t voltage_dv;
    uint8_t temp_c;
    uint8_t status;
    uint8_t moving;
    int16_t current_ma;
} sb_feedback_t;

/* Sign-magnitude decode used by STS registers: sign_bit is the bit index of the sign. */
int32_t sb_sign_mag_decode(uint16_t raw, unsigned sign_bit);
uint16_t sb_sign_mag_encode(int32_t v, unsigned sign_bit);
bool sb_decode_feedback(const uint8_t *params, size_t n, sb_feedback_t *fb);

/* Round-robin read scheduler: one feedback READ per control cycle. */
typedef struct {
    uint8_t n;          /* number of servos */
    uint8_t next;       /* index polled next */
    int8_t pending;     /* index polled last cycle awaiting a reply, -1 if none */
    uint8_t miss[ARMFW_N_JOINTS];
} sb_sched_t;

void sb_sched_init(sb_sched_t *s, uint8_t n);
/* Index to poll this cycle (advances the round robin and marks it pending). */
uint8_t sb_sched_next(sb_sched_t *s);
/* Resolve the pending poll: answered or not. Returns the index, or -1 if nothing was pending. */
int sb_sched_resolve(sb_sched_t *s, bool answered);

/* Bus time (us) needed for a cycle: tx_bytes written + reply_bytes read + turnaround allowance. */
uint32_t sb_cycle_bus_us(size_t tx_bytes, size_t reply_bytes, uint32_t turnaround_us);

#endif
