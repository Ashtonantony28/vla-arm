/* libarmfw.so C API, exercised through the shared library exactly as a host tool would. */
#include <stdalign.h>

#include "armfw_api.h"
#include "tt.h"

TT_TEST(versions_and_crc) {
    TT_EQ(armfw_api_version(), 1);
    TT_EQ(armfw_fw_version(), 0x0100);
    TT_EQ(armfw_crc16((const uint8_t *)"123456789", 9), 0x29B1);
    TT_EQ(armfw_expected_len(0x82), 66);
    TT_EQ(armfw_expected_len(0x42), -1);
    TT_EQ(armfw_state_sizeof(), 68);
}

TT_TEST(encode_parse_roundtrip_with_chunked_feed) {
    alignas(16) static uint8_t parser[1024];
    TT_ASSERT(armfw_parser_sizeof() <= sizeof parser);
    armfw_parser_init(parser);
    uint8_t stream[2048];
    size_t pos = 0;
    int16_t p6[6] = {1, -2, 3, -4, 5, 1600};
    uint8_t pl[240];
    for (int k = 0; k < 20; k++) {
        size_t n = armfw_pack_set_targets((uint16_t)(10 * k + 1), p6, 1, pl);
        TT_EQ(n, 15);
        pos += armfw_encode_frame(0x02, (uint8_t)k, pl, n, &stream[pos], sizeof stream - pos);
        stream[pos++] = 0x00; /* garbage between frames */
    }
    armfw_frame_t frames[3];
    size_t off = 0;
    int got = 0;
    while (off < pos) {
        size_t used = 0;
        size_t nf = armfw_parser_feed(parser, &stream[off], pos - off, frames, 3, &used);
        for (size_t i = 0; i < nf; i++) {
            uint16_t ms;
            int16_t q[6];
            uint8_t fl;
            TT_ASSERT(armfw_unpack_set_targets(frames[i].payload, frames[i].len, &ms, q, &fl));
            TT_EQ(frames[i].seq, got);
            TT_EQ(ms, 10 * got + 1);
            TT_EQ(q[5], 1600);
            got++;
        }
        off += used;
    }
    TT_EQ(got, 20);
    uint32_t st[6];
    armfw_parser_stats(parser, st);
    TT_EQ(st[0], 20);
    TT_EQ(st[4], 20); /* the 20 garbage bytes */
}

TT_TEST(all_payload_helpers) {
    uint8_t b[66];
    uint8_t j, k, s, t, e, pv;
    int32_t v;
    uint16_t m, fw;
    uint32_t up;
    TT_EQ(armfw_pack_config(3, 0x05, -70000, b), 6);
    TT_ASSERT(armfw_unpack_config(b, 6, &j, &k, &v));
    TT_EQ(j, 3); TT_EQ(k, 5); TT_EQ(v, -70000);
    TT_EQ(armfw_pack_clear_fault(0x00FF, b), 2);
    TT_ASSERT(armfw_unpack_clear_fault(b, 2, &m));
    TT_EQ(m, 0xFF);
    TT_EQ(armfw_pack_set_stream(5000, b), 2);
    TT_ASSERT(armfw_unpack_set_stream(b, 2, &m));
    TT_EQ(m, 5000);
    TT_EQ(armfw_pack_pong(0x0100, 123456, 1, b), 7);
    TT_ASSERT(armfw_unpack_pong(b, 7, &fw, &up, &pv));
    TT_EQ(fw, 0x0100); TT_EQ(up, 123456); TT_EQ(pv, 1);
    TT_EQ(armfw_pack_ack(9, 2, b), 2);
    TT_ASSERT(armfw_unpack_ack(b, 2, &s, &t));
    TT_EQ(s, 9); TT_EQ(t, 2);
    TT_EQ(armfw_pack_nack(9, 3, 5, b), 3);
    TT_ASSERT(armfw_unpack_nack(b, 3, &s, &t, &e));
    TT_EQ(e, 5);
    TT_ASSERT(!armfw_unpack_nack(b, 2, &s, &t, &e));
    armfw_state_t st, st2;
    memset(&st, 0, sizeof st);
    st.t_us = 77; st.mode = 1; st.pos_mrad[5] = -1234; st.joint_status[2] = 0x42; st.cmd_pos_crc = 0xBEEF;
    TT_EQ(armfw_pack_state(&st, b), 66);
    TT_ASSERT(armfw_unpack_state(b, 66, &st2));
    TT_EQ(st2.t_us, 77); TT_EQ(st2.pos_mrad[5], -1234); TT_EQ(st2.joint_status[2], 0x42); TT_EQ(st2.cmd_pos_crc, 0xBEEF);
    TT_ASSERT(!armfw_unpack_state(b, 65, &st2));
}

/* Drive the in-process firmware sim through the API only. */
TT_TEST(sim_through_api) {
    size_t sz = armfw_sim_sizeof();
    uint8_t *mem = aligned_alloc(16, (sz + 15) / 16 * 16);
    TT_ASSERT(mem != NULL);
    armfw_sim_init(mem, 0);
    armfw_sim_step(mem, 20);
    uint8_t fr[64], pl[16];
    int16_t tgt[6] = {500, -300, 200, 100, -800, 1200};
    size_t n = armfw_encode_frame(0x02, 1, pl, armfw_pack_set_targets(400, tgt, 1, pl), fr, sizeof fr);
    TT_EQ(armfw_sim_host_write(mem, fr, n), n);
    for (int i = 0; i < 10; i++) {
        n = armfw_encode_frame(0x01, (uint8_t)(2 + i), NULL, 0, fr, sizeof fr);
        armfw_sim_host_write(mem, fr, n);
        armfw_sim_step(mem, 90);
    }
    TT_EQ(armfw_sim_mode(mem), 1);
    TT_EQ(armfw_sim_faults(mem), 0);
    TT_EQ(armfw_sim_time_us(mem), 920000);
    for (int j = 0; j < 6; j++) TT_NEAR(armfw_sim_servo_pos_mrad(mem, j), tgt[j], 2.0);
    /* read the board's output and find a STATE */
    static uint8_t out[65536];
    size_t got = armfw_sim_host_read(mem, out, sizeof out);
    TT_ASSERT(got > 0);
    alignas(16) static uint8_t parser[1024];
    armfw_parser_init(parser);
    static armfw_frame_t frames[400];
    size_t used = 0, nf = armfw_parser_feed(parser, out, got, frames, 400, &used);
    int states = 0, pongs = 0, acks = 0;
    armfw_state_t st;
    for (size_t i = 0; i < nf; i++) {
        if (frames[i].type == 0x82) {
            states++;
            TT_ASSERT(armfw_unpack_state(frames[i].payload, frames[i].len, &st));
        }
        pongs += frames[i].type == 0x81;
        acks += frames[i].type == 0x83;
    }
    TT_EQ(states, 184); /* 920 ms at 200 Hz */
    TT_EQ(pongs, 10);
    TT_EQ(acks, 1);
    TT_EQ(st.mode, 1);
    TT_EQ(st.last_host_seq, 11);
    /* fault injection */
    armfw_sim_set_servo_temp(mem, 2, 80);
    armfw_sim_step(mem, 10);
    TT_EQ(armfw_sim_mode(mem), 3);
    TT_EQ(armfw_sim_faults(mem), 0x10);
    free(mem);
}

TT_MAIN()
