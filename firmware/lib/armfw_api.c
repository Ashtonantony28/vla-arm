#include "armfw_api.h"

#include <string.h>

#include "armfw/config.h"
#include "armfw/crc16.h"
#include "armfw/proto.h"
#include "sim.h"

_Static_assert(sizeof(armfw_frame_t) == sizeof(proto_frame_t), "frame layout");
_Static_assert(sizeof(armfw_frame_t) == 244, "frame size");
_Static_assert(offsetof(armfw_state_t, pos_mrad) == 8, "state layout");
_Static_assert(offsetof(armfw_state_t, temp_c) == 44, "state layout");
_Static_assert(offsetof(armfw_state_t, bus_mv) == 56, "state layout");
_Static_assert(offsetof(armfw_state_t, cmd_pos_crc) == 64, "state layout");

uint32_t armfw_api_version(void) { return ARMFW_API_VERSION; }
uint16_t armfw_fw_version(void) { return ARMFW_FW_VERSION; }
uint16_t armfw_crc16(const uint8_t *data, size_t len) { return crc16_ccitt_false(data, len); }

size_t armfw_encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t len, uint8_t *out,
                          size_t out_cap) {
    return proto_encode(out, out_cap, type, seq, payload, len);
}

int armfw_expected_len(uint8_t type) { return proto_expected_len(type); }

/* ---- parser ---- */

size_t armfw_parser_sizeof(void) { return sizeof(proto_parser_t); }
void armfw_parser_init(void *parser) { proto_parser_init((proto_parser_t *)parser); }

size_t armfw_parser_feed(void *parser, const uint8_t *data, size_t len, armfw_frame_t *out, size_t max_frames,
                         size_t *consumed) {
    proto_parser_t *p = (proto_parser_t *)parser;
    size_t nf = 0, i = 0;
    proto_frame_t f;
    /* drain frames still buffered from a previous call first */
    while (nf < max_frames && proto_parser_poll(p, &f)) memcpy(&out[nf++], &f, sizeof f);
    for (; i < len && nf < max_frames; i++) {
        proto_parser_feed(p, data[i]);
        while (nf < max_frames && proto_parser_poll(p, &f)) memcpy(&out[nf++], &f, sizeof f);
    }
    if (consumed) *consumed = i;
    return nf;
}

void armfw_parser_stats(const void *parser, uint32_t out[6]) {
    const proto_parser_stats_t *s = &((const proto_parser_t *)parser)->stats;
    out[0] = s->frames_ok;
    out[1] = s->crc_errors;
    out[2] = s->ver_errors;
    out[3] = s->len_errors;
    out[4] = s->bytes_skipped;
    out[5] = s->overflows;
}

/* ---- payloads ---- */

size_t armfw_pack_set_targets(uint16_t move_ms, const int16_t pos[6], uint8_t flags, uint8_t *out) {
    proto_set_targets_t m;
    m.move_ms = move_ms;
    memcpy(m.pos, pos, sizeof m.pos);
    m.flags = flags;
    return proto_pack_set_targets(&m, out);
}
int armfw_unpack_set_targets(const uint8_t *p, size_t len, uint16_t *move_ms, int16_t pos[6], uint8_t *flags) {
    proto_set_targets_t m;
    if (!proto_unpack_set_targets(p, len, &m)) return 0;
    *move_ms = m.move_ms;
    memcpy(pos, m.pos, sizeof m.pos);
    *flags = m.flags;
    return 1;
}

size_t armfw_pack_config(uint8_t joint, uint8_t key, int32_t value, uint8_t *out) {
    proto_config_t m = {joint, key, value};
    return proto_pack_config(&m, out);
}
int armfw_unpack_config(const uint8_t *p, size_t len, uint8_t *joint, uint8_t *key, int32_t *value) {
    proto_config_t m;
    if (!proto_unpack_config(p, len, &m)) return 0;
    *joint = m.joint;
    *key = m.key;
    *value = m.value;
    return 1;
}

size_t armfw_pack_clear_fault(uint16_t mask, uint8_t *out) {
    proto_clear_fault_t m = {mask};
    return proto_pack_clear_fault(&m, out);
}
int armfw_unpack_clear_fault(const uint8_t *p, size_t len, uint16_t *mask) {
    proto_clear_fault_t m;
    if (!proto_unpack_clear_fault(p, len, &m)) return 0;
    *mask = m.mask;
    return 1;
}

size_t armfw_pack_set_stream(uint16_t period_us, uint8_t *out) {
    proto_set_stream_t m = {period_us};
    return proto_pack_set_stream(&m, out);
}
int armfw_unpack_set_stream(const uint8_t *p, size_t len, uint16_t *period_us) {
    proto_set_stream_t m;
    if (!proto_unpack_set_stream(p, len, &m)) return 0;
    *period_us = m.period_us;
    return 1;
}

size_t armfw_pack_pong(uint16_t fw_version, uint32_t uptime_ms, uint8_t proto_ver, uint8_t *out) {
    proto_pong_t m = {fw_version, uptime_ms, proto_ver};
    return proto_pack_pong(&m, out);
}
int armfw_unpack_pong(const uint8_t *p, size_t len, uint16_t *fw_version, uint32_t *uptime_ms,
                      uint8_t *proto_ver) {
    proto_pong_t m;
    if (!proto_unpack_pong(p, len, &m)) return 0;
    *fw_version = m.fw_version;
    *uptime_ms = m.uptime_ms;
    *proto_ver = m.proto_ver;
    return 1;
}

size_t armfw_pack_ack(uint8_t seq, uint8_t type, uint8_t *out) {
    proto_ack_t m = {seq, type};
    return proto_pack_ack(&m, out);
}
int armfw_unpack_ack(const uint8_t *p, size_t len, uint8_t *seq, uint8_t *type) {
    proto_ack_t m;
    if (!proto_unpack_ack(p, len, &m)) return 0;
    *seq = m.seq;
    *type = m.type;
    return 1;
}

size_t armfw_pack_nack(uint8_t seq, uint8_t type, uint8_t error, uint8_t *out) {
    proto_nack_t m = {seq, type, error};
    return proto_pack_nack(&m, out);
}
int armfw_unpack_nack(const uint8_t *p, size_t len, uint8_t *seq, uint8_t *type, uint8_t *error) {
    proto_nack_t m;
    if (!proto_unpack_nack(p, len, &m)) return 0;
    *seq = m.seq;
    *type = m.type;
    *error = m.error;
    return 1;
}

size_t armfw_state_sizeof(void) { return sizeof(armfw_state_t); }

size_t armfw_pack_state(const armfw_state_t *st, uint8_t *out) {
    proto_state_t m;
    m.t_us = st->t_us;
    m.fault_flags = st->fault_flags;
    m.mode = st->mode;
    m.last_host_seq = st->last_host_seq;
    for (int j = 0; j < 6; j++) {
        m.joint[j].pos_mrad = st->pos_mrad[j];
        m.joint[j].vel_mrad_s = st->vel_mrad_s[j];
        m.joint[j].current_ma = st->current_ma[j];
        m.joint[j].temp_c = st->temp_c[j];
        m.joint[j].status = st->joint_status[j];
    }
    m.bus_mv = st->bus_mv;
    m.bus_ma = st->bus_ma;
    m.loop_jitter_max_us = st->loop_jitter_max_us;
    m.loop_overruns = st->loop_overruns;
    m.cmd_pos_crc = st->cmd_pos_crc;
    return proto_pack_state(&m, out);
}

int armfw_unpack_state(const uint8_t *p, size_t len, armfw_state_t *st) {
    proto_state_t m;
    if (!proto_unpack_state(p, len, &m)) return 0;
    memset(st, 0, sizeof(*st));
    st->t_us = m.t_us;
    st->fault_flags = m.fault_flags;
    st->mode = m.mode;
    st->last_host_seq = m.last_host_seq;
    for (int j = 0; j < 6; j++) {
        st->pos_mrad[j] = m.joint[j].pos_mrad;
        st->vel_mrad_s[j] = m.joint[j].vel_mrad_s;
        st->current_ma[j] = m.joint[j].current_ma;
        st->temp_c[j] = m.joint[j].temp_c;
        st->joint_status[j] = m.joint[j].status;
    }
    st->bus_mv = m.bus_mv;
    st->bus_ma = m.bus_ma;
    st->loop_jitter_max_us = m.loop_jitter_max_us;
    st->loop_overruns = m.loop_overruns;
    st->cmd_pos_crc = m.cmd_pos_crc;
    return 1;
}

/* ---- sim ---- */

static bool joint_ok(int j) { return j >= 0 && j < ARMFW_N_JOINTS; }

size_t armfw_sim_sizeof(void) { return sizeof(sim_t); }
void armfw_sim_init(void *sim, uint32_t start_us) { sim_init((sim_t *)sim, start_us); }
void armfw_sim_step(void *sim, uint32_t n) { sim_run((sim_t *)sim, n); }
uint32_t armfw_sim_time_us(const void *sim) { return ((const sim_t *)sim)->now_us; }
size_t armfw_sim_host_write(void *sim, const uint8_t *data, size_t len) {
    return sim_host_send((sim_t *)sim, data, len);
}
size_t armfw_sim_host_read(void *sim, uint8_t *buf, size_t cap) { return sim_host_recv((sim_t *)sim, buf, cap); }
void armfw_sim_set_estop_closed(void *sim, int closed) { ((sim_t *)sim)->estop_closed = closed != 0; }
void armfw_sim_set_supply_mv(void *sim, uint32_t mv) { ((sim_t *)sim)->supply_v = (float)mv * 1e-3f; }
void armfw_sim_set_servo_silent(void *sim, int joint, int silent) {
    if (joint_ok(joint)) ((sim_t *)sim)->servo[joint].silent = silent != 0;
}
void armfw_sim_set_servo_temp(void *sim, int joint, int temp_c) {
    if (joint_ok(joint)) ((sim_t *)sim)->servo[joint].temp_c = (float)temp_c;
}
void armfw_sim_inject_current(void *sim, int joint, int enable, int current_ma) {
    if (!joint_ok(joint)) return;
    fake_servo_t *s = &((sim_t *)sim)->servo[joint];
    s->inject_current = enable != 0;
    s->current_override_ma = (int16_t)armfw_clampi(current_ma, -32768, 32767);
}
void armfw_sim_set_echo(void *sim, int echo) { ((sim_t *)sim)->echo_tx = echo != 0; }
int armfw_sim_mode(const void *sim) { return (int)((const sim_t *)sim)->ctl.safety.mode; }
uint16_t armfw_sim_faults(const void *sim) { return ((const sim_t *)sim)->ctl.safety.faults; }
float armfw_sim_servo_pos_mrad(const void *sim, int joint) {
    if (!joint_ok(joint)) return 0.0f;
    const sim_t *s = (const sim_t *)sim;
    return config_ticks_to_mrad(&s->ctl.cfg.joint[joint], (uint16_t)(s->servo[joint].pos_ticks + 0.5f));
}
