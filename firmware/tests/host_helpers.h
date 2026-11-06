/* Host-side helpers for closed-loop tests: send frames to the sim, collect the board's frames. */
#ifndef HOST_HELPERS_H
#define HOST_HELPERS_H

#include "armfw/crc16.h"
#include "armfw/proto.h"
#include "sim.h"
#include "tt.h"

#define HH_MAX_FRAMES 8192

typedef struct {
    sim_t *sim;
    uint8_t seq;
    proto_parser_t parser;
    proto_frame_t frames[HH_MAX_FRAMES];
    uint32_t frame_step[HH_MAX_FRAMES]; /* sim step at which the frame was received */
    int n;
    uint32_t step;
} host_t;

static inline void hh_init(host_t *h, sim_t *s) {
    memset(h, 0, sizeof *h);
    h->sim = s;
    proto_parser_init(&h->parser);
}

static inline void hh_drain(host_t *h) {
    uint8_t buf[512];
    size_t n;
    proto_frame_t f;
    while ((n = sim_host_recv(h->sim, buf, sizeof buf)) > 0) {
        for (size_t i = 0; i < n; i++) {
            proto_parser_feed(&h->parser, buf[i]);
            while (proto_parser_poll(&h->parser, &f)) {
                if (h->n < HH_MAX_FRAMES) {
                    h->frame_step[h->n] = h->step;
                    h->frames[h->n++] = f;
                }
            }
        }
    }
}

static inline void hh_run(host_t *h, uint32_t steps) {
    for (uint32_t i = 0; i < steps; i++) {
        sim_step(h->sim);
        h->step++;
        hh_drain(h);
    }
}

static inline uint8_t hh_send(host_t *h, uint8_t type, const uint8_t *pl, size_t len) {
    uint8_t buf[PROTO_MAX_FRAME];
    uint8_t seq = h->seq++;
    size_t n = proto_encode(buf, sizeof buf, type, seq, pl, len);
    sim_host_send(h->sim, buf, n);
    return seq;
}

static inline uint8_t hh_set_targets(host_t *h, uint16_t move_ms, const int16_t pos[6], uint8_t flags) {
    proto_set_targets_t m = {.move_ms = move_ms, .flags = flags};
    for (int j = 0; j < 6; j++) m.pos[j] = pos[j];
    uint8_t pl[PROTO_LEN_SET_TARGETS];
    proto_pack_set_targets(&m, pl);
    return hh_send(h, PROTO_T_SET_TARGETS, pl, sizeof pl);
}

static inline uint8_t hh_config(host_t *h, uint8_t joint, uint8_t key, int32_t value) {
    proto_config_t m = {joint, key, value};
    uint8_t pl[PROTO_LEN_CONFIG];
    proto_pack_config(&m, pl);
    return hh_send(h, PROTO_T_CONFIG, pl, sizeof pl);
}

static inline uint8_t hh_clear(host_t *h, uint16_t mask) {
    proto_clear_fault_t m = {mask};
    uint8_t pl[2];
    proto_pack_clear_fault(&m, pl);
    return hh_send(h, PROTO_T_CLEAR_FAULT, pl, 2);
}

static inline uint8_t hh_stream(host_t *h, uint16_t period) {
    proto_set_stream_t m = {period};
    uint8_t pl[2];
    proto_pack_set_stream(&m, pl);
    return hh_send(h, PROTO_T_SET_STREAM, pl, 2);
}

/* Find the reply (ACK/NACK) to a given seq; returns 0 = none, 1 = ACK, -err = NACK code. */
static inline int hh_reply(const host_t *h, uint8_t seq, uint8_t type) {
    for (int i = h->n - 1; i >= 0; i--) {
        const proto_frame_t *f = &h->frames[i];
        if (f->type == PROTO_T_ACK && f->payload[0] == seq && f->payload[1] == type) return 1;
        if (f->type == PROTO_T_NACK && f->payload[0] == seq && f->payload[1] == type) return -(int)f->payload[2];
    }
    return 0;
}

static inline bool hh_last_state(const host_t *h, proto_state_t *st) {
    for (int i = h->n - 1; i >= 0; i--)
        if (h->frames[i].type == PROTO_T_STATE) return proto_unpack_state(h->frames[i].payload, h->frames[i].len, st);
    return false;
}

static inline int hh_count(const host_t *h, uint8_t type) {
    int c = 0;
    for (int i = 0; i < h->n; i++) c += h->frames[i].type == type;
    return c;
}

static inline void hh_clear_frames(host_t *h) { h->n = 0; }

#endif
