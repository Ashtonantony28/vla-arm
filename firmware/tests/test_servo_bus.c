/* Servo bus: packet encoding against published example packets, parser, feedback decode,
 * round-robin scheduler and the per-cycle bus time budget. */
#include "armfw/servo_bus.h"
#include "tt.h"

TT_TEST(known_packet_ping) {
    /* Dynamixel 1.0 / Feetech manual: PING ID 1 -> FF FF 01 02 01 FB */
    uint8_t out[16];
    size_t n = sb_encode_ping(out, sizeof out, 1);
    uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
    TT_EQ(n, sizeof exp);
    TT_MEM_EQ(out, exp, n);
}

TT_TEST(known_packet_read) {
    /* AX-12 manual example: read 1 byte at 0x2B from ID 1 -> FF FF 01 04 02 2B 01 CC */
    uint8_t out[16];
    size_t n = sb_encode_read(out, sizeof out, 1, 0x2B, 1);
    uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC};
    TT_EQ(n, sizeof exp);
    TT_MEM_EQ(out, exp, n);
}

TT_TEST(known_packet_write_feetech_goal) {
    /* Feetech STS example: ID 1, write 0x2A: pos 2048, time 0, speed 1000
     * -> FF FF 01 09 03 2A 00 08 00 00 E8 03 D5 */
    uint8_t out[32];
    uint8_t data[] = {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03};
    size_t n = sb_encode_write(out, sizeof out, 1, 0x2A, data, sizeof data);
    uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 0xD5};
    TT_EQ(n, sizeof exp);
    TT_MEM_EQ(out, exp, n);
}

TT_TEST(known_packet_broadcast_write) {
    /* AX-12 manual: broadcast write ID register (0x03) = 1 -> FF FF FE 04 03 03 01 F6 */
    uint8_t out[16];
    uint8_t d = 1;
    size_t n = sb_encode_write(out, sizeof out, SB_BROADCAST_ID, 0x03, &d, 1);
    uint8_t exp[] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0xF6};
    TT_MEM_EQ(out, exp, n);
}

TT_TEST(known_packet_sync_write) {
    /* AX-12 manual SYNC_WRITE example: 4 servos, addr 0x1E, 4 bytes each */
    uint8_t ids[] = {0, 1, 2, 3};
    uint8_t data[] = {0x10, 0x00, 0x50, 0x01, 0x20, 0x02, 0x60, 0x03,
                      0x30, 0x00, 0x70, 0x01, 0x20, 0x02, 0x80, 0x03};
    uint8_t out[64];
    size_t n = sb_encode_sync_write(out, sizeof out, 0x1E, 4, ids, data, 4);
    uint8_t exp[] = {0xFF, 0xFF, 0xFE, 0x18, 0x83, 0x1E, 0x04, 0x00, 0x10, 0x00, 0x50, 0x01, 0x01, 0x20,
                     0x02, 0x60, 0x03, 0x02, 0x30, 0x00, 0x70, 0x01, 0x03, 0x20, 0x02, 0x80, 0x03, 0x12};
    TT_EQ(n, sizeof exp);
    TT_MEM_EQ(out, exp, n);
}

TT_TEST(known_packet_status) {
    uint8_t out[16];
    size_t n = sb_encode_status(out, sizeof out, 1, 0, NULL, 0);
    uint8_t exp[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    TT_MEM_EQ(out, exp, n);
    /* AX-12 manual: status reply with temperature 0x20 -> FF FF 01 03 00 20 DB */
    uint8_t t = 0x20;
    n = sb_encode_status(out, sizeof out, 1, 0, &t, 1);
    uint8_t exp2[] = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB};
    TT_EQ(n, sizeof exp2);
    TT_MEM_EQ(out, exp2, n);
}

TT_TEST(sync_goal_layout_and_checksum) {
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    uint16_t ticks[6] = {2048, 0, 4095, 1000, 3000, 2100};
    uint8_t out[64];
    size_t n = sb_encode_sync_goal(out, sizeof out, ids, ticks, 6);
    TT_EQ(n, 26); /* 6 header/len/instr/chk + addr + len + 6*(id+2) */
    TT_EQ(out[2], 0xFE);
    TT_EQ(out[3], 22);
    TT_EQ(out[4], 0x83);
    TT_EQ(out[5], 0x2A);
    TT_EQ(out[6], 2);
    TT_EQ(out[7], 1);
    TT_EQ(out[8], 0x00);
    TT_EQ(out[9], 0x08);
    TT_EQ(out[13], 3);
    TT_EQ(out[14], 0xFF);
    TT_EQ(out[15], 0x0F);
    TT_EQ(out[25], sb_checksum(&out[2], 23));
    n = sb_encode_sync_torque(out, sizeof out, ids, 6, true);
    TT_EQ(n, 20);
    TT_EQ(out[5], 0x28);
    TT_EQ(out[6], 1);
    for (int i = 0; i < 6; i++) {
        TT_EQ(out[7 + 2 * i], i + 1);
        TT_EQ(out[8 + 2 * i], 1);
    }
}

TT_TEST(encode_capacity_guards) {
    uint8_t out[8];
    TT_EQ(sb_encode_read(out, 7, 1, 0x38, 15), 0);
    TT_EQ(sb_encode_read(out, 8, 1, 0x38, 15), 8);
    uint8_t big[70] = {0};
    uint8_t o2[128];
    TT_EQ(sb_encode(o2, sizeof o2, 1, 3, big, 65), 0);
}

TT_TEST(parser_roundtrip_and_resync) {
    uint8_t stream[256];
    size_t pos = 0;
    stream[pos++] = 0x00;
    stream[pos++] = 0xFF; /* stray header byte */
    uint8_t params[15];
    for (int i = 0; i < 15; i++) params[i] = (uint8_t)(i * 7);
    pos += sb_encode_status(&stream[pos], sizeof stream - pos, 3, 0, params, 15);
    /* corrupted packet */
    size_t bad = pos;
    pos += sb_encode_status(&stream[pos], sizeof stream - pos, 4, 0, params, 15);
    stream[bad + 7] ^= 0x40;
    /* FF FF FF prefix then a valid packet */
    stream[pos++] = 0xFF;
    pos += sb_encode_status(&stream[pos], sizeof stream - pos, 5, 0x20, params, 2);
    sb_parser_t p;
    sb_parser_init(&p);
    sb_packet_t pk[8];
    int n = 0;
    for (size_t i = 0; i < pos; i++) {
        sb_parser_feed(&p, stream[i]);
        while (n < 8 && sb_parser_poll(&p, &pk[n])) n++;
    }
    TT_EQ(n, 2);
    TT_EQ(pk[0].id, 3);
    TT_EQ(pk[0].nparams, 15);
    TT_MEM_EQ(pk[0].params, params, 15);
    TT_EQ(pk[1].id, 5);
    TT_EQ(pk[1].code, 0x20);
    TT_EQ(pk[1].nparams, 2);
    TT_EQ(p.stats.checksum_errors, 1);
}

TT_TEST(sign_magnitude) {
    TT_EQ(sb_sign_mag_decode(0x8005, 15), -5);
    TT_EQ(sb_sign_mag_decode(0x0005, 15), 5);
    TT_EQ(sb_sign_mag_decode(0x0400 | 300, 10), -300);
    TT_EQ(sb_sign_mag_decode(300, 10), 300);
    for (int v = -1000; v <= 1000; v += 7) {
        TT_EQ(sb_sign_mag_decode(sb_sign_mag_encode(v, 15), 15), v);
        TT_EQ(sb_sign_mag_decode(sb_sign_mag_encode(v, 10), 10), v);
    }
    TT_EQ(sb_sign_mag_encode(5000, 10), 1023); /* saturates */
}

TT_TEST(feedback_decode) {
    uint8_t r[15] = {0};
    armfw_put_u16(&r[0], 3000);
    armfw_put_u16(&r[2], sb_sign_mag_encode(-250, 15));
    armfw_put_u16(&r[4], sb_sign_mag_encode(-400, 10));
    r[6] = 121;
    r[7] = 44;
    r[9] = 0x20;
    r[10] = 1;
    armfw_put_u16(&r[13], sb_sign_mag_encode(-100, 15));
    sb_feedback_t fb;
    TT_ASSERT(sb_decode_feedback(r, 15, &fb));
    TT_EQ(fb.pos_ticks, 3000);
    TT_EQ(fb.speed_ticks_s, -250);
    TT_EQ(fb.load_permille, -400);
    TT_EQ(fb.voltage_dv, 121);
    TT_EQ(fb.temp_c, 44);
    TT_EQ(fb.status, 0x20);
    TT_EQ(fb.moving, 1);
    TT_EQ(fb.current_ma, -650);
    TT_ASSERT(!sb_decode_feedback(r, 14, &fb));
}

TT_TEST(round_robin_scheduler) {
    sb_sched_t s;
    sb_sched_init(&s, 6);
    TT_EQ(sb_sched_resolve(&s, true), -1);
    int counts[6] = {0};
    for (int k = 0; k < 600; k++) {
        uint8_t i = sb_sched_next(&s);
        TT_EQ(i, k % 6);
        counts[i]++;
        /* servo 2 never answers */
        TT_EQ(sb_sched_resolve(&s, i != 2), i);
    }
    for (int i = 0; i < 6; i++) TT_EQ(counts[i], 100); /* each joint at 1000/6 ~ 166 Hz */
    TT_EQ(s.miss[2], 100);
    TT_EQ(s.miss[1], 0);
}

TT_TEST(bus_budget_under_1ms) {
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    uint16_t t[6] = {0};
    uint8_t out[128];
    size_t goal = sb_encode_sync_goal(out, sizeof out, ids, t, 6);
    size_t rd = sb_encode_read(out, sizeof out, 1, SB_FEEDBACK_ADDR, SB_FEEDBACK_LEN);
    size_t reply = 6 + SB_FEEDBACK_LEN;
    size_t tq = sb_encode_sync_torque(out, sizeof out, ids, 6, true);
    /* turnaround: servo return delay (configured 0..20 us) + direction switch, budget 50 us */
    uint32_t normal = sb_cycle_bus_us(goal + rd, reply, 50);
    uint32_t enable = sb_cycle_bus_us(tq + goal + rd, reply, 50);
    printf("    bus per cycle: normal %u us (%zu B tx, %zu B rx), torque-enable cycle %u us\n", normal,
           goal + rd, reply, enable);
    TT_ASSERT(normal < 1000);
    TT_ASSERT(enable < 1000);
}

TT_MAIN()
