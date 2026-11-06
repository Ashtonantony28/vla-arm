#include "armfw/servo_bus.h"

#include <string.h>

uint8_t sb_checksum(const uint8_t *from_id, size_t n) {
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) sum += from_id[i];
    return (uint8_t)(~sum & 0xFFu);
}

static size_t encode_raw(uint8_t *out, size_t cap, uint8_t id, uint8_t code, const uint8_t *params,
                         size_t nparams) {
    if (nparams > SB_MAX_PARAMS || cap < nparams + 6u) return 0;
    out[0] = SB_HDR0;
    out[1] = SB_HDR1;
    out[2] = id;
    out[3] = (uint8_t)(nparams + 2u);
    out[4] = code;
    if (nparams) memcpy(&out[5], params, nparams);
    out[5 + nparams] = sb_checksum(&out[2], nparams + 3u);
    return nparams + 6u;
}

size_t sb_encode(uint8_t *out, size_t cap, uint8_t id, uint8_t instr, const uint8_t *params,
                 size_t nparams) {
    return encode_raw(out, cap, id, instr, params, nparams);
}

size_t sb_encode_status(uint8_t *out, size_t cap, uint8_t id, uint8_t error, const uint8_t *params,
                        size_t nparams) {
    return encode_raw(out, cap, id, error, params, nparams);
}

size_t sb_encode_ping(uint8_t *out, size_t cap, uint8_t id) {
    return encode_raw(out, cap, id, SB_INST_PING, NULL, 0);
}

size_t sb_encode_read(uint8_t *out, size_t cap, uint8_t id, uint8_t addr, uint8_t len) {
    uint8_t p[2] = {addr, len};
    return encode_raw(out, cap, id, SB_INST_READ, p, 2);
}

size_t sb_encode_write(uint8_t *out, size_t cap, uint8_t id, uint8_t addr, const uint8_t *data,
                       size_t n) {
    uint8_t p[SB_MAX_PARAMS];
    if (n + 1u > SB_MAX_PARAMS) return 0;
    p[0] = addr;
    memcpy(&p[1], data, n);
    return encode_raw(out, cap, id, SB_INST_WRITE, p, n + 1u);
}

size_t sb_encode_sync_write(uint8_t *out, size_t cap, uint8_t addr, uint8_t data_len,
                            const uint8_t *ids, const uint8_t *data, size_t n_ids) {
    uint8_t p[SB_MAX_PARAMS];
    size_t np = 2u + n_ids * (1u + data_len);
    if (np > SB_MAX_PARAMS) return 0;
    p[0] = addr;
    p[1] = data_len;
    size_t k = 2;
    for (size_t i = 0; i < n_ids; i++) {
        p[k++] = ids[i];
        memcpy(&p[k], &data[i * data_len], data_len);
        k += data_len;
    }
    return encode_raw(out, cap, SB_BROADCAST_ID, SB_INST_SYNC_WRITE, p, np);
}

size_t sb_encode_sync_goal(uint8_t *out, size_t cap, const uint8_t *ids, const uint16_t *ticks,
                           size_t n) {
    uint8_t d[2 * ARMFW_N_JOINTS * 2];
    if (n > 2 * ARMFW_N_JOINTS) return 0;
    for (size_t i = 0; i < n; i++) armfw_put_u16(&d[2 * i], ticks[i]);
    return sb_encode_sync_write(out, cap, SB_REG_GOAL_POSITION, 2, ids, d, n);
}

size_t sb_encode_sync_torque(uint8_t *out, size_t cap, const uint8_t *ids, size_t n, bool enable) {
    uint8_t d[2 * ARMFW_N_JOINTS];
    if (n > 2 * ARMFW_N_JOINTS) return 0;
    for (size_t i = 0; i < n; i++) d[i] = enable ? 1u : 0u;
    return sb_encode_sync_write(out, cap, SB_REG_TORQUE_ENABLE, 1, ids, d, n);
}

/* ---------------- streaming parser ---------------- */

void sb_parser_init(sb_parser_t *p) { memset(p, 0, sizeof(*p)); }

bool sb_parser_feed(sb_parser_t *p, uint8_t b) {
    if (p->len >= SB_MAX_PACKET) {
        p->stats.overflows++;
        return false;
    }
    p->buf[p->len++] = b;
    return true;
}

static void sb_skip(sb_parser_t *p) {
    uint16_t n = 1;
    while (n < p->len && p->buf[n] != SB_HDR0) n++;
    p->stats.bytes_skipped += n;
    memmove(p->buf, &p->buf[n], (size_t)(p->len - n));
    p->len = (uint16_t)(p->len - n);
}

bool sb_parser_poll(sb_parser_t *p, sb_packet_t *out) {
    for (;;) {
        if (p->len == 0) return false;
        if (p->buf[0] != SB_HDR0) { sb_skip(p); continue; }
        if (p->len < 2) return false;
        if (p->buf[1] != SB_HDR1) { sb_skip(p); continue; }
        if (p->len < 3) return false;
        /* ID 0xFF is not valid: treat FF FF FF as a shifted header */
        if (p->buf[2] == 0xFFu) { sb_skip(p); continue; }
        if (p->len < 4) return false;
        uint8_t ln = p->buf[3];
        if (ln < 2u || ln > SB_MAX_PARAMS + 2u) { p->stats.len_errors++; sb_skip(p); continue; }
        uint16_t need = (uint16_t)(ln + 4u);
        if (p->len < need) return false;
        if (sb_checksum(&p->buf[2], (size_t)ln + 1u) != p->buf[need - 1]) {
            p->stats.checksum_errors++;
            sb_skip(p);
            continue;
        }
        out->id = p->buf[2];
        out->code = p->buf[4];
        out->nparams = (uint8_t)(ln - 2u);
        if (out->nparams) memcpy(out->params, &p->buf[5], out->nparams);
        memmove(p->buf, &p->buf[need], (size_t)(p->len - need));
        p->len = (uint16_t)(p->len - need);
        p->stats.packets_ok++;
        return true;
    }
}

/* ---------------- feedback decode ---------------- */

int32_t sb_sign_mag_decode(uint16_t raw, unsigned sign_bit) {
    uint16_t mag_mask = (uint16_t)((1u << sign_bit) - 1u);
    int32_t mag = (int32_t)(raw & mag_mask);
    return (raw & (1u << sign_bit)) ? -mag : mag;
}

uint16_t sb_sign_mag_encode(int32_t v, unsigned sign_bit) {
    uint32_t mag_mask = (1u << sign_bit) - 1u;
    uint32_t mag = (uint32_t)(v < 0 ? -v : v);
    if (mag > mag_mask) mag = mag_mask;
    return (uint16_t)(mag | (v < 0 ? (1u << sign_bit) : 0u));
}

bool sb_decode_feedback(const uint8_t *params, size_t n, sb_feedback_t *fb) {
    if (n != SB_FEEDBACK_LEN) return false;
    const uint8_t *r = params; /* r[k] = register 0x38 + k */
    int32_t pos = sb_sign_mag_decode(armfw_get_u16(&r[0]), 15);
    fb->pos_ticks = (uint16_t)armfw_clampi(pos, 0, 4095); /* single-turn position mode */
    fb->speed_ticks_s = (int16_t)sb_sign_mag_decode(armfw_get_u16(&r[2]), 15);
    fb->load_permille = (int16_t)sb_sign_mag_decode(armfw_get_u16(&r[4]), 10);
    fb->voltage_dv = r[6];
    fb->temp_c = r[7];
    fb->status = r[9];
    fb->moving = r[10];
    int32_t cur = sb_sign_mag_decode(armfw_get_u16(&r[13]), 15);
    float ma = (float)cur * SB_CURRENT_MA_PER_LSB;
    fb->current_ma = armfw_f2i16(ma);
    return true;
}

/* ---------------- scheduler ---------------- */

void sb_sched_init(sb_sched_t *s, uint8_t n) {
    memset(s, 0, sizeof(*s));
    s->n = n > ARMFW_N_JOINTS ? ARMFW_N_JOINTS : n;
    s->pending = -1;
}

uint8_t sb_sched_next(sb_sched_t *s) {
    uint8_t i = s->next;
    s->next = (uint8_t)((s->next + 1u) % s->n);
    s->pending = (int8_t)i;
    return i;
}

int sb_sched_resolve(sb_sched_t *s, bool answered) {
    int i = s->pending;
    if (i < 0) return -1;
    if (answered) {
        s->miss[i] = 0;
    } else if (s->miss[i] < 255) {
        s->miss[i]++;
    }
    s->pending = -1;
    return i;
}

uint32_t sb_cycle_bus_us(size_t tx_bytes, size_t reply_bytes, uint32_t turnaround_us) {
    return (uint32_t)((tx_bytes + reply_bytes) * SB_US_PER_BYTE_X10 / 10u) + turnaround_us;
}
