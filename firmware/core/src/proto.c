#include "armfw/proto.h"

#include <string.h>

#include "armfw/crc16.h"

int proto_expected_len(uint8_t type) {
    switch (type) {
    case PROTO_T_PING: return PROTO_LEN_PING;
    case PROTO_T_SET_TARGETS: return PROTO_LEN_SET_TARGETS;
    case PROTO_T_CONFIG: return PROTO_LEN_CONFIG;
    case PROTO_T_ESTOP: return PROTO_LEN_ESTOP;
    case PROTO_T_CLEAR_FAULT: return PROTO_LEN_CLEAR_FAULT;
    case PROTO_T_SET_STREAM: return PROTO_LEN_SET_STREAM;
    case PROTO_T_PONG: return PROTO_LEN_PONG;
    case PROTO_T_STATE: return PROTO_LEN_STATE;
    case PROTO_T_ACK: return PROTO_LEN_ACK;
    case PROTO_T_NACK: return PROTO_LEN_NACK;
    default: return -1;
    }
}

bool proto_is_host_type(uint8_t type) { return type >= PROTO_T_PING && type <= PROTO_T_SET_STREAM; }

size_t proto_encode(uint8_t *out, size_t cap, uint8_t type, uint8_t seq, const uint8_t *payload,
                    size_t len) {
    if (len > PROTO_MAX_PAYLOAD || cap < len + PROTO_OVERHEAD) return 0;
    out[0] = PROTO_SYNC0;
    out[1] = PROTO_SYNC1;
    out[2] = PROTO_VER;
    out[3] = type;
    out[4] = seq;
    out[5] = (uint8_t)len;
    if (len) memcpy(&out[6], payload, len);
    uint16_t crc = crc16_ccitt_false(&out[2], 4 + len);
    armfw_put_u16(&out[6 + len], crc);
    return len + PROTO_OVERHEAD;
}

/* ---------------- pack / unpack ---------------- */

size_t proto_pack_set_targets(const proto_set_targets_t *m, uint8_t *out) {
    armfw_put_u16(out, m->move_ms);
    for (int i = 0; i < ARMFW_N_JOINTS; i++) armfw_put_i16(&out[2 + 2 * i], m->pos[i]);
    out[14] = m->flags;
    return PROTO_LEN_SET_TARGETS;
}
bool proto_unpack_set_targets(const uint8_t *p, size_t len, proto_set_targets_t *m) {
    if (len != PROTO_LEN_SET_TARGETS) return false;
    m->move_ms = armfw_get_u16(p);
    for (int i = 0; i < ARMFW_N_JOINTS; i++) m->pos[i] = armfw_get_i16(&p[2 + 2 * i]);
    m->flags = p[14];
    return true;
}

size_t proto_pack_config(const proto_config_t *m, uint8_t *out) {
    out[0] = m->joint;
    out[1] = m->key;
    armfw_put_i32(&out[2], m->value);
    return PROTO_LEN_CONFIG;
}
bool proto_unpack_config(const uint8_t *p, size_t len, proto_config_t *m) {
    if (len != PROTO_LEN_CONFIG) return false;
    m->joint = p[0];
    m->key = p[1];
    m->value = armfw_get_i32(&p[2]);
    return true;
}

size_t proto_pack_clear_fault(const proto_clear_fault_t *m, uint8_t *out) {
    armfw_put_u16(out, m->mask);
    return PROTO_LEN_CLEAR_FAULT;
}
bool proto_unpack_clear_fault(const uint8_t *p, size_t len, proto_clear_fault_t *m) {
    if (len != PROTO_LEN_CLEAR_FAULT) return false;
    m->mask = armfw_get_u16(p);
    return true;
}

size_t proto_pack_set_stream(const proto_set_stream_t *m, uint8_t *out) {
    armfw_put_u16(out, m->period_us);
    return PROTO_LEN_SET_STREAM;
}
bool proto_unpack_set_stream(const uint8_t *p, size_t len, proto_set_stream_t *m) {
    if (len != PROTO_LEN_SET_STREAM) return false;
    m->period_us = armfw_get_u16(p);
    return true;
}

size_t proto_pack_pong(const proto_pong_t *m, uint8_t *out) {
    armfw_put_u16(out, m->fw_version);
    armfw_put_u32(&out[2], m->uptime_ms);
    out[6] = m->proto_ver;
    return PROTO_LEN_PONG;
}
bool proto_unpack_pong(const uint8_t *p, size_t len, proto_pong_t *m) {
    if (len != PROTO_LEN_PONG) return false;
    m->fw_version = armfw_get_u16(p);
    m->uptime_ms = armfw_get_u32(&p[2]);
    m->proto_ver = p[6];
    return true;
}

size_t proto_pack_state(const proto_state_t *m, uint8_t *out) {
    armfw_put_u32(&out[0], m->t_us);
    armfw_put_u16(&out[4], m->fault_flags);
    out[6] = m->mode;
    out[7] = m->last_host_seq;
    for (int i = 0; i < ARMFW_N_JOINTS; i++) {
        uint8_t *j = &out[8 + 8 * i];
        armfw_put_i16(&j[0], m->joint[i].pos_mrad);
        armfw_put_i16(&j[2], m->joint[i].vel_mrad_s);
        armfw_put_i16(&j[4], m->joint[i].current_ma);
        j[6] = m->joint[i].temp_c;
        j[7] = m->joint[i].status;
    }
    armfw_put_u16(&out[56], m->bus_mv);
    armfw_put_i16(&out[58], m->bus_ma);
    armfw_put_u16(&out[60], m->loop_jitter_max_us);
    armfw_put_u16(&out[62], m->loop_overruns);
    armfw_put_u16(&out[64], m->cmd_pos_crc);
    return PROTO_LEN_STATE;
}
bool proto_unpack_state(const uint8_t *p, size_t len, proto_state_t *m) {
    if (len != PROTO_LEN_STATE) return false;
    m->t_us = armfw_get_u32(&p[0]);
    m->fault_flags = armfw_get_u16(&p[4]);
    m->mode = p[6];
    m->last_host_seq = p[7];
    for (int i = 0; i < ARMFW_N_JOINTS; i++) {
        const uint8_t *j = &p[8 + 8 * i];
        m->joint[i].pos_mrad = armfw_get_i16(&j[0]);
        m->joint[i].vel_mrad_s = armfw_get_i16(&j[2]);
        m->joint[i].current_ma = armfw_get_i16(&j[4]);
        m->joint[i].temp_c = j[6];
        m->joint[i].status = j[7];
    }
    m->bus_mv = armfw_get_u16(&p[56]);
    m->bus_ma = armfw_get_i16(&p[58]);
    m->loop_jitter_max_us = armfw_get_u16(&p[60]);
    m->loop_overruns = armfw_get_u16(&p[62]);
    m->cmd_pos_crc = armfw_get_u16(&p[64]);
    return true;
}

size_t proto_pack_ack(const proto_ack_t *m, uint8_t *out) {
    out[0] = m->seq;
    out[1] = m->type;
    return PROTO_LEN_ACK;
}
bool proto_unpack_ack(const uint8_t *p, size_t len, proto_ack_t *m) {
    if (len != PROTO_LEN_ACK) return false;
    m->seq = p[0];
    m->type = p[1];
    return true;
}

size_t proto_pack_nack(const proto_nack_t *m, uint8_t *out) {
    out[0] = m->seq;
    out[1] = m->type;
    out[2] = m->error;
    return PROTO_LEN_NACK;
}
bool proto_unpack_nack(const uint8_t *p, size_t len, proto_nack_t *m) {
    if (len != PROTO_LEN_NACK) return false;
    m->seq = p[0];
    m->type = p[1];
    m->error = p[2];
    return true;
}

/* ---------------- streaming parser ---------------- */

void proto_parser_init(proto_parser_t *p) { memset(p, 0, sizeof(*p)); }

bool proto_parser_feed(proto_parser_t *p, uint8_t b) {
    if (p->len >= PROTO_MAX_FRAME) {
        p->stats.overflows++;
        return false;
    }
    p->buf[p->len++] = b;
    return true;
}

/* Drop the first byte and then everything up to the next SYNC0 candidate. */
static void skip_one(proto_parser_t *p) {
    uint16_t n = 1;
    while (n < p->len && p->buf[n] != PROTO_SYNC0) n++;
    p->stats.bytes_skipped += n;
    memmove(p->buf, &p->buf[n], (size_t)(p->len - n));
    p->len = (uint16_t)(p->len - n);
}

bool proto_parser_poll(proto_parser_t *p, proto_frame_t *out) {
    for (;;) {
        if (p->len == 0) return false;
        if (p->buf[0] != PROTO_SYNC0) { skip_one(p); continue; }
        if (p->len < 2) return false;
        if (p->buf[1] != PROTO_SYNC1) { skip_one(p); continue; }
        if (p->len < 3) return false;
        if (p->buf[2] != PROTO_VER) { p->stats.ver_errors++; skip_one(p); continue; }
        if (p->len < PROTO_HEADER_LEN) return false;
        uint8_t plen = p->buf[5];
        if (plen > PROTO_MAX_PAYLOAD) { p->stats.len_errors++; skip_one(p); continue; }
        uint16_t need = (uint16_t)(plen + PROTO_OVERHEAD);
        if (p->len < need) return false;
        uint16_t crc = crc16_ccitt_false(&p->buf[2], (size_t)4 + plen);
        if (crc != armfw_get_u16(&p->buf[6 + plen])) { p->stats.crc_errors++; skip_one(p); continue; }
        out->ver = p->buf[2];
        out->type = p->buf[3];
        out->seq = p->buf[4];
        out->len = plen;
        if (plen) memcpy(out->payload, &p->buf[6], plen);
        memmove(p->buf, &p->buf[need], (size_t)(p->len - need));
        p->len = (uint16_t)(p->len - need);
        p->stats.frames_ok++;
        return true;
    }
}

size_t proto_parser_push(proto_parser_t *p, const uint8_t *data, size_t n, proto_frame_cb cb,
                         void *ctx) {
    size_t count = 0;
    proto_frame_t f;
    for (size_t i = 0; i < n; i++) {
        proto_parser_feed(p, data[i]);
        while (proto_parser_poll(p, &f)) {
            count++;
            if (cb) cb(ctx, &f);
        }
    }
    return count;
}
