/* Protocol: CRC, frame encoding, payload round trips, streaming parser resync. */
#include "armfw/crc16.h"
#include "armfw/proto.h"
#include "tt.h"

typedef struct {
    proto_frame_t frames[64];
    int n;
} sink_t;

static void sink_cb(void *ctx, const proto_frame_t *f) {
    sink_t *s = (sink_t *)ctx;
    if (s->n < 64) s->frames[s->n++] = *f;
}

TT_TEST(crc_check_value) {
    const char *s = "123456789";
    TT_EQ(crc16_ccitt_false((const uint8_t *)s, 9), 0x29B1);
    TT_EQ(crc16_ccitt_false(NULL, 0), 0xFFFF);
    /* incremental update equals one-shot */
    uint16_t c = crc16_update(CRC16_INIT, (const uint8_t *)s, 4);
    c = crc16_update(c, (const uint8_t *)s + 4, 5);
    TT_EQ(c, 0x29B1);
}

TT_TEST(crc_matches_bitwise_reference) {
    tt_rng_t r = {0x1234567ULL};
    uint8_t buf[300];
    for (int iter = 0; iter < 200; iter++) {
        size_t n = (size_t)tt_randi(&r, 0, 300);
        for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)tt_rand(&r);
        uint16_t ref = 0xFFFF;
        for (size_t i = 0; i < n; i++) {
            ref ^= (uint16_t)(buf[i] << 8);
            for (int b = 0; b < 8; b++) ref = (ref & 0x8000) ? (uint16_t)((ref << 1) ^ 0x1021) : (uint16_t)(ref << 1);
        }
        TT_EQ(crc16_ccitt_false(buf, n), ref);
    }
}

TT_TEST(encode_ping_known_bytes) {
    uint8_t out[16];
    size_t n = proto_encode(out, sizeof out, PROTO_T_PING, 7, NULL, 0);
    TT_EQ(n, 8);
    uint8_t hdr[] = {0x01, 0x01, 0x07, 0x00};
    uint16_t crc = crc16_ccitt_false(hdr, 4);
    uint8_t exp[] = {0xA5, 0x5A, 0x01, 0x01, 0x07, 0x00, (uint8_t)crc, (uint8_t)(crc >> 8)};
    TT_MEM_EQ(out, exp, 8);
    /* capacity / length guards */
    TT_EQ(proto_encode(out, 7, PROTO_T_PING, 0, NULL, 0), 0);
    uint8_t big[256] = {0};
    uint8_t obuf[300];
    TT_EQ(proto_encode(obuf, sizeof obuf, 0x55, 0, big, 241), 0);
    TT_EQ(proto_encode(obuf, sizeof obuf, 0x55, 0, big, 240), 248);
}

TT_TEST(expected_lengths_match_spec) {
    TT_EQ(proto_expected_len(PROTO_T_PING), 0);
    TT_EQ(proto_expected_len(PROTO_T_SET_TARGETS), 15);
    TT_EQ(proto_expected_len(PROTO_T_CONFIG), 6);
    TT_EQ(proto_expected_len(PROTO_T_ESTOP), 0);
    TT_EQ(proto_expected_len(PROTO_T_CLEAR_FAULT), 2);
    TT_EQ(proto_expected_len(PROTO_T_SET_STREAM), 2);
    TT_EQ(proto_expected_len(PROTO_T_PONG), 7);
    TT_EQ(proto_expected_len(PROTO_T_STATE), 66);
    TT_EQ(proto_expected_len(PROTO_T_ACK), 2);
    TT_EQ(proto_expected_len(PROTO_T_NACK), 3);
    TT_EQ(proto_expected_len(0x00), -1);
    TT_EQ(proto_expected_len(0x07), -1);
    TT_EQ(proto_expected_len(0x80), -1);
}

/* Encode payload -> frame -> parser -> unpack, for every message type. */
static proto_frame_t roundtrip_frame(uint8_t type, uint8_t seq, const uint8_t *pl, size_t n) {
    uint8_t buf[PROTO_MAX_FRAME];
    size_t fl = proto_encode(buf, sizeof buf, type, seq, pl, n);
    TT_EQ(fl, n + 8);
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    TT_EQ(proto_parser_push(&p, buf, fl, sink_cb, &s), 1);
    TT_EQ(s.frames[0].type, type);
    TT_EQ(s.frames[0].seq, seq);
    TT_EQ(s.frames[0].len, n);
    TT_EQ(s.frames[0].ver, 1);
    TT_EQ(p.len, 0);
    return s.frames[0];
}

TT_TEST(roundtrip_all_message_types) {
    uint8_t pl[PROTO_MAX_PAYLOAD];
    proto_frame_t f;

    f = roundtrip_frame(PROTO_T_PING, 0, NULL, 0);
    f = roundtrip_frame(PROTO_T_ESTOP, 255, NULL, 0);

    proto_set_targets_t st = {.move_ms = 5000, .pos = {-32768, 32767, 0, -1, 1234, 1600}, .flags = 1}, st2;
    size_t n = proto_pack_set_targets(&st, pl);
    TT_EQ(n, 15);
    TT_EQ(pl[0], 0x88); TT_EQ(pl[1], 0x13); /* 5000 LE */
    TT_EQ(pl[2], 0x00); TT_EQ(pl[3], 0x80); /* -32768 LE */
    f = roundtrip_frame(PROTO_T_SET_TARGETS, 3, pl, n);
    TT_ASSERT(proto_unpack_set_targets(f.payload, f.len, &st2));
    TT_EQ(st2.move_ms, 5000);
    for (int i = 0; i < 6; i++) TT_EQ(st2.pos[i], st.pos[i]);
    TT_EQ(st2.flags, 1);
    TT_ASSERT(!proto_unpack_set_targets(f.payload, 14, &st2));

    proto_config_t c = {.joint = 0xFF, .key = 0x11, .value = -123456789}, c2;
    n = proto_pack_config(&c, pl);
    TT_EQ(n, 6);
    f = roundtrip_frame(PROTO_T_CONFIG, 4, pl, n);
    TT_ASSERT(proto_unpack_config(f.payload, f.len, &c2));
    TT_EQ(c2.joint, 0xFF); TT_EQ(c2.key, 0x11); TT_EQ(c2.value, -123456789);
    TT_ASSERT(!proto_unpack_config(f.payload, 7, &c2));

    proto_clear_fault_t cf = {.mask = 0xBEEF}, cf2;
    n = proto_pack_clear_fault(&cf, pl);
    f = roundtrip_frame(PROTO_T_CLEAR_FAULT, 5, pl, n);
    TT_ASSERT(proto_unpack_clear_fault(f.payload, f.len, &cf2));
    TT_EQ(cf2.mask, 0xBEEF);

    proto_set_stream_t ss = {.period_us = 5000}, ss2;
    n = proto_pack_set_stream(&ss, pl);
    f = roundtrip_frame(PROTO_T_SET_STREAM, 6, pl, n);
    TT_ASSERT(proto_unpack_set_stream(f.payload, f.len, &ss2));
    TT_EQ(ss2.period_us, 5000);

    proto_pong_t po = {.fw_version = 0x0102, .uptime_ms = 0xDEADBEEF, .proto_ver = 1}, po2;
    n = proto_pack_pong(&po, pl);
    TT_EQ(n, 7);
    f = roundtrip_frame(PROTO_T_PONG, 7, pl, n);
    TT_ASSERT(proto_unpack_pong(f.payload, f.len, &po2));
    TT_EQ(po2.fw_version, 0x0102); TT_EQ(po2.uptime_ms, 0xDEADBEEFu); TT_EQ(po2.proto_ver, 1);

    proto_state_t s, s2;
    memset(&s, 0, sizeof s);
    s.t_us = 0x01020304; s.fault_flags = 0x00A5; s.mode = 2; s.last_host_seq = 200;
    for (int i = 0; i < 6; i++) {
        s.joint[i].pos_mrad = (int16_t)(-1000 * i + 7);
        s.joint[i].vel_mrad_s = (int16_t)(300 * i - 900);
        s.joint[i].current_ma = (int16_t)(-50 * i);
        s.joint[i].temp_c = (uint8_t)(30 + i);
        s.joint[i].status = (uint8_t)(1 << i);
    }
    s.bus_mv = 12034; s.bus_ma = -321; s.loop_jitter_max_us = 17; s.loop_overruns = 65535; s.cmd_pos_crc = 0x1234;
    n = proto_pack_state(&s, pl);
    TT_EQ(n, 66);
    TT_EQ(pl[0], 0x04); TT_EQ(pl[3], 0x01); TT_EQ(pl[6], 2); TT_EQ(pl[7], 200);
    TT_EQ(pl[64], 0x34); TT_EQ(pl[65], 0x12);
    f = roundtrip_frame(PROTO_T_STATE, 8, pl, n);
    TT_ASSERT(proto_unpack_state(f.payload, f.len, &s2));
    TT_EQ(s2.t_us, s.t_us); TT_EQ(s2.fault_flags, s.fault_flags); TT_EQ(s2.mode, 2); TT_EQ(s2.last_host_seq, 200);
    for (int i = 0; i < 6; i++) {
        TT_EQ(s2.joint[i].pos_mrad, s.joint[i].pos_mrad);
        TT_EQ(s2.joint[i].vel_mrad_s, s.joint[i].vel_mrad_s);
        TT_EQ(s2.joint[i].current_ma, s.joint[i].current_ma);
        TT_EQ(s2.joint[i].temp_c, s.joint[i].temp_c);
        TT_EQ(s2.joint[i].status, s.joint[i].status);
    }
    TT_EQ(s2.bus_mv, 12034); TT_EQ(s2.bus_ma, -321); TT_EQ(s2.loop_jitter_max_us, 17);
    TT_EQ(s2.loop_overruns, 65535); TT_EQ(s2.cmd_pos_crc, 0x1234);
    TT_ASSERT(!proto_unpack_state(f.payload, 65, &s2));

    proto_ack_t a = {.seq = 9, .type = 2}, a2;
    n = proto_pack_ack(&a, pl);
    f = roundtrip_frame(PROTO_T_ACK, 9, pl, n);
    TT_ASSERT(proto_unpack_ack(f.payload, f.len, &a2));
    TT_EQ(a2.seq, 9); TT_EQ(a2.type, 2);

    proto_nack_t na = {.seq = 10, .type = 3, .error = 5}, na2;
    n = proto_pack_nack(&na, pl);
    f = roundtrip_frame(PROTO_T_NACK, 10, pl, n);
    TT_ASSERT(proto_unpack_nack(f.payload, f.len, &na2));
    TT_EQ(na2.seq, 10); TT_EQ(na2.type, 3); TT_EQ(na2.error, 5);

    /* max payload frame */
    for (int i = 0; i < 240; i++) pl[i] = (uint8_t)i;
    f = roundtrip_frame(0x42, 11, pl, 240);
    TT_MEM_EQ(f.payload, pl, 240);
}

static size_t mk_frame(uint8_t *out, uint8_t type, uint8_t seq, uint8_t n) {
    uint8_t pl[PROTO_MAX_PAYLOAD] = {0};
    for (int i = 0; i < n; i++) pl[i] = (uint8_t)(seq + i);
    return proto_encode(out, PROTO_MAX_FRAME, type, seq, pl, n);
}

TT_TEST(parser_resync_on_garbage) {
    tt_rng_t r = {42};
    uint8_t stream[8192];
    size_t pos = 0;
    int expect = 0;
    for (int k = 0; k < 40; k++) {
        /* random garbage, including stray sync bytes */
        int g = tt_randi(&r, 0, 30);
        for (int i = 0; i < g; i++) {
            uint32_t v = tt_rand(&r) % 8;
            stream[pos++] = v == 0 ? 0xA5 : (v == 1 ? 0x5A : (uint8_t)tt_rand(&r));
        }
        pos += mk_frame(&stream[pos], PROTO_T_SET_STREAM, (uint8_t)k, 2);
        expect++;
    }
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    size_t got = proto_parser_push(&p, stream, pos, sink_cb, &s);
    /* every real frame must come out in order; random garbage could, with ~2^-16 odds per
     * candidate, form an extra valid frame - with this seed it does not. */
    TT_EQ(got, (size_t)expect);
    for (int k = 0; k < expect; k++) TT_EQ(s.frames[k].seq, k);
    TT_ASSERT(p.stats.bytes_skipped > 0);
}

TT_TEST(parser_split_frames_byte_by_byte) {
    uint8_t stream[2048];
    size_t pos = 0;
    for (int k = 0; k < 10; k++) pos += mk_frame(&stream[pos], 0x10, (uint8_t)k, (uint8_t)(k * 20));
    /* feed in chunks of random size */
    tt_rng_t r = {7};
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    size_t i = 0;
    while (i < pos) {
        size_t c = (size_t)tt_randi(&r, 1, 13);
        if (i + c > pos) c = pos - i;
        proto_parser_push(&p, &stream[i], c, sink_cb, &s);
        i += c;
    }
    TT_EQ(s.n, 10);
    for (int k = 0; k < 10; k++) {
        TT_EQ(s.frames[k].seq, k);
        TT_EQ(s.frames[k].len, k * 20);
        for (int b = 0; b < k * 20; b++) TT_EQ(s.frames[k].payload[b], (uint8_t)(k + b));
    }
    TT_EQ(p.stats.crc_errors, 0);
    TT_EQ(p.stats.bytes_skipped, 0);
}

TT_TEST(parser_bad_crc_dropped_next_frame_found) {
    uint8_t stream[64];
    size_t a = mk_frame(stream, 0x01, 1, 4);
    stream[a - 1] ^= 0x01; /* corrupt CRC of frame 1 */
    size_t b = mk_frame(&stream[a], 0x01, 2, 4);
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, a + b, sink_cb, &s);
    TT_EQ(s.n, 1);
    TT_EQ(s.frames[0].seq, 2);
    TT_EQ(p.stats.crc_errors, 1);
}

TT_TEST(parser_bad_payload_byte_is_crc_error) {
    uint8_t stream[64];
    size_t a = mk_frame(stream, 0x01, 1, 6);
    stream[8] ^= 0x80;
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, a, sink_cb, &s);
    TT_EQ(s.n, 0);
    TT_EQ(p.stats.crc_errors, 1);
}

TT_TEST(parser_bad_len_rejected_quickly) {
    /* header with LEN = 241 must be rejected as soon as LEN is seen, and the following frame parsed */
    uint8_t stream[64] = {0xA5, 0x5A, 0x01, 0x02, 0x00, 241};
    size_t b = mk_frame(&stream[6], 0x02, 9, 3);
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, 6 + b, sink_cb, &s);
    TT_EQ(s.n, 1);
    TT_EQ(s.frames[0].seq, 9);
    TT_EQ(p.stats.len_errors, 1);
}

TT_TEST(parser_bad_version_rejected) {
    uint8_t stream[64];
    size_t a = mk_frame(stream, 0x01, 1, 2);
    stream[2] = 0x02; /* VER 2 (CRC also covers VER, but VER check comes first) */
    size_t b = mk_frame(&stream[a], 0x01, 2, 2);
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, a + b, sink_cb, &s);
    TT_EQ(s.n, 1);
    TT_EQ(s.frames[0].seq, 2);
    TT_EQ(p.stats.ver_errors, 1);
}

TT_TEST(parser_frame_hidden_inside_corrupt_frame) {
    /* A truncated frame claiming LEN=200 followed by real frames: once the 208 bytes are in and
     * the CRC fails, the parser must rescan and recover the embedded frames. */
    uint8_t stream[512] = {0xA5, 0x5A, 0x01, 0x10, 0x00, 200};
    size_t pos = 6;
    for (int k = 0; k < 5; k++) pos += mk_frame(&stream[pos], 0x01, (uint8_t)(100 + k), 10);
    while (pos < 6 + 200 + 2 + 20) stream[pos++] = 0x00;
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, pos, sink_cb, &s);
    TT_EQ(s.n, 5);
    for (int k = 0; k < 5; k++) TT_EQ(s.frames[k].seq, 100 + k);
    TT_EQ(p.stats.crc_errors, 1);
}

TT_TEST(parser_double_sync_prefix) {
    uint8_t stream[64] = {0xA5, 0xA5, 0xA5};
    size_t a = mk_frame(&stream[3], 0x01, 77, 0);
    proto_parser_t p;
    proto_parser_init(&p);
    sink_t s = {.n = 0};
    proto_parser_push(&p, stream, 3 + a, sink_cb, &s);
    TT_EQ(s.n, 1);
    TT_EQ(s.frames[0].seq, 77);
}

TT_TEST(parser_random_corruption_never_yields_wrong_frame) {
    /* Property: under random single-byte corruption, every frame the parser outputs is
     * byte-identical to one that was sent (CRC-16 false-accept aside, checked by content). */
    tt_rng_t r = {99};
    for (int iter = 0; iter < 300; iter++) {
        uint8_t stream[1024];
        size_t pos = 0;
        int nf = tt_randi(&r, 1, 6);
        for (int k = 0; k < nf; k++) pos += mk_frame(&stream[pos], 0x05, (uint8_t)k, (uint8_t)tt_randi(&r, 0, 60));
        int nc = tt_randi(&r, 0, 3);
        for (int c = 0; c < nc; c++) stream[tt_rand(&r) % pos] ^= (uint8_t)(1u << (tt_rand(&r) % 8));
        proto_parser_t p;
        proto_parser_init(&p);
        sink_t s = {.n = 0};
        proto_parser_push(&p, stream, pos, sink_cb, &s);
        TT_ASSERT(s.n <= nf);
        for (int k = 0; k < s.n; k++) {
            const proto_frame_t *f = &s.frames[k];
            TT_EQ(f->type, 0x05);
            for (int b = 0; b < f->len; b++) TT_EQ(f->payload[b], (uint8_t)(f->seq + b));
        }
        if (nc == 0) TT_EQ(s.n, nf);
    }
}

TT_TEST(parser_overflow_when_not_polled) {
    proto_parser_t p;
    proto_parser_init(&p);
    /* a frame header claiming 240 bytes, then fill without polling */
    uint8_t hdr[6] = {0xA5, 0x5A, 0x01, 0x01, 0x00, 240};
    for (int i = 0; i < 6; i++) TT_ASSERT(proto_parser_feed(&p, hdr[i]));
    for (int i = 0; i < 242; i++) TT_ASSERT(proto_parser_feed(&p, 0));
    TT_ASSERT(!proto_parser_feed(&p, 0));
    TT_EQ(p.stats.overflows, 1);
    proto_frame_t f;
    TT_ASSERT(!proto_parser_poll(&p, &f)); /* bad CRC, rescans, nothing valid */
    TT_EQ(p.stats.crc_errors, 1);
}

TT_MAIN()
