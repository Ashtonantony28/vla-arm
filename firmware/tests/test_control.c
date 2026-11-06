/* Closed-loop tests: the real control core + fake servos + fake HAL, driven through host frames
 * exactly as the host driver would. */
#include <math.h>
#include <stdlib.h>

#include "armfw/config.h"
#include "host_helpers.h"

static sim_t g_sim;
static host_t g_host;

static host_t *boot(void) {
    sim_init(&g_sim, 1000);
    hh_init(&g_host, &g_sim);
    hh_run(&g_host, 20); /* every servo polled at least once */
    return &g_host;
}

static const int16_t k_home[6] = {0, 0, 0, 0, 0, 800};

static host_t *boot_active(void) {
    host_t *h = boot();
    uint8_t s = hh_set_targets(h, 200, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    return h;
}

/* Keep the watchdog fed with PINGs while running n ms. */
static void run_fed(host_t *h, int n) {
    for (int i = 0; i < n; i++) {
        if (i % 50 == 0) hh_send(h, PROTO_T_PING, NULL, 0);
        hh_run(h, 1);
    }
}

TT_TEST(boot_state_stream_idle) {
    host_t *h = boot();
    hh_clear_frames(h);
    hh_run(h, 1000);
    TT_EQ(hh_count(h, PROTO_T_STATE), 200);
    proto_state_t st;
    TT_ASSERT(hh_last_state(h, &st));
    TT_EQ(st.mode, PROTO_MODE_IDLE);
    TT_EQ(st.fault_flags, 0);
    TT_NEAR(st.bus_mv, 12000, 100);
    for (int j = 0; j < 6; j++) {
        TT_ASSERT(st.joint[j].status & JS_FEEDBACK_OK);
        TT_ASSERT(!(st.joint[j].status & JS_TORQUE_ON));
        TT_NEAR(st.joint[j].pos_mrad, k_home[j], 2);
        TT_EQ(st.joint[j].temp_c, 35);
    }
    /* torque-off is (re)sent to the servos while idle */
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
    /* STATE timestamps are 5000 us apart */
    uint32_t prev = 0;
    int k = 0;
    for (int i = 0; i < h->n; i++) {
        if (h->frames[i].type != PROTO_T_STATE) continue;
        proto_unpack_state(h->frames[i].payload, h->frames[i].len, &st);
        if (k++ > 0) TT_EQ(st.t_us - prev, 5000);
        prev = st.t_us;
    }
}

TT_TEST(ping_pong) {
    host_t *h = boot();
    hh_send(h, PROTO_T_PING, NULL, 0);
    hh_run(h, 1);
    for (int i = 0; i < h->n; i++) {
        if (h->frames[i].type != PROTO_T_PONG) continue;
        proto_pong_t p;
        TT_ASSERT(proto_unpack_pong(h->frames[i].payload, h->frames[i].len, &p));
        TT_EQ(p.fw_version, ARMFW_FW_VERSION);
        TT_EQ(p.proto_ver, 1);
        TT_EQ(p.uptime_ms, 21);
        return;
    }
    TT_FAIL("no PONG");
}

TT_TEST(set_targets_before_feedback_is_rejected) {
    sim_init(&g_sim, 0);
    hh_init(&g_host, &g_sim);
    hh_run(&g_host, 2);
    uint8_t s = hh_set_targets(&g_host, 100, k_home, PROTO_FLAG_TORQUE);
    hh_run(&g_host, 1);
    TT_EQ(hh_reply(&g_host, s, PROTO_T_SET_TARGETS), -PROTO_ERR_FAULTED);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
}

/* The headline closed-loop test: 5 s of a 30 Hz policy streaming sinusoidal targets, then a final
 * target held. Joints must track and converge; STATE must arrive at exactly 200 Hz. */
TT_TEST(closed_loop_5s_converges_state_200hz) {
    host_t *h = boot();
    hh_clear_frames(h);
    uint32_t t0 = h->step;
    int16_t tgt[6];
    int sent = 0;
    uint8_t seqs[256];
    double max_track_err = 0;
    for (int ms = 0; ms < 5000; ms++) {
        if (ms % 33 == 0 && ms < 4000) {
            double t = ms * 1e-3;
            tgt[0] = (int16_t)(800 * sin(2 * 3.14159265358979 * 0.3 * t));
            tgt[1] = (int16_t)(-400 + 300 * sin(2 * 3.14159265358979 * 0.2 * t));
            tgt[2] = (int16_t)(600 * sin(2 * 3.14159265358979 * 0.25 * t + 1));
            tgt[3] = (int16_t)(500 * cos(2 * 3.14159265358979 * 0.3 * t) - 500);
            tgt[4] = (int16_t)(1000 * sin(2 * 3.14159265358979 * 0.1 * t));
            tgt[5] = (int16_t)(800 + 600 * sin(2 * 3.14159265358979 * 0.5 * t));
            seqs[sent++ % 256] = hh_set_targets(h, ms == 0 ? 1000 : 66, tgt, PROTO_FLAG_TORQUE);
        }
        if (ms == 4000) {
            static const int16_t final_tgt[6] = {300, -500, 700, -200, 1500, 1200};
            memcpy(tgt, final_tgt, sizeof tgt);
            seqs[sent++ % 256] = hh_set_targets(h, 500, tgt, PROTO_FLAG_TORQUE);
        }
        if (ms > 4000 && ms % 50 == 0) hh_send(h, PROTO_T_PING, NULL, 0);
        hh_run(h, 1);
        if (ms > 1500 && ms < 4000) {
            /* measured follows the command with the servo lag (tau 30 ms + 1 cycle) */
            for (int j = 0; j < 6; j++) {
                double e = fabs(g_sim.ctl.meas[j].pos_mrad - g_sim.ctl.traj.cmd[j].p);
                if (e > max_track_err) max_track_err = e;
            }
        }
    }
    uint32_t elapsed = h->step - t0;
    TT_EQ(elapsed, 5000);
    int nstate = hh_count(h, PROTO_T_STATE);
    printf("    %d STATE frames in 5.000 s, %d SET_TARGETS acked, max tracking error %.1f mrad\n", nstate,
           hh_count(h, PROTO_T_ACK), max_track_err);
    TT_EQ(nstate, 1000); /* 200 Hz */
    TT_EQ(hh_count(h, PROTO_T_ACK), sent);
    TT_EQ(hh_count(h, PROTO_T_NACK), 0);
    TT_ASSERT(max_track_err < 150);
    /* STATE frames evenly spaced: every 5 steps */
    int prev = -1;
    for (int i = 0; i < h->n; i++) {
        if (h->frames[i].type != PROTO_T_STATE) continue;
        if (prev >= 0) TT_EQ(h->frame_step[i] - h->frame_step[prev], 5);
        prev = i;
    }
    proto_state_t st;
    TT_ASSERT(hh_last_state(h, &st));
    TT_EQ(st.mode, PROTO_MODE_ACTIVE);
    TT_EQ(st.fault_flags & ~FAULT_LIMIT_CLAMP, 0);
    TT_EQ(st.last_host_seq, (uint8_t)(h->seq - 1));
    for (int j = 0; j < 6; j++) {
        /* converged within one servo tick (1.53 mrad) + rounding */
        TT_ASSERT_MSG(abs(st.joint[j].pos_mrad - tgt[j]) <= 2, "joint %d at %d, target %d", j,
                      st.joint[j].pos_mrad, tgt[j]);
        TT_ASSERT(st.joint[j].status & JS_TORQUE_ON);
        TT_ASSERT(!(st.joint[j].status & JS_MOVING));
        TT_EQ(st.joint[j].vel_mrad_s, 0);
        TT_ASSERT(abs(st.joint[j].current_ma) >= 100); /* holding current */
    }
    TT_ASSERT(st.bus_ma > 500);
    TT_EQ(st.loop_overruns, 0);
    /* cmd_pos_crc matches the commanded (= final target) positions */
    uint8_t b[12];
    for (int j = 0; j < 6; j++) armfw_put_i16(&b[2 * j], tgt[j]);
    TT_EQ(st.cmd_pos_crc, crc16_ccitt_false(b, 12));
    (void)seqs;
}

TT_TEST(watchdog_hold_then_fault_via_frames) {
    host_t *h = boot_active();
    static const int16_t t1[6] = {500, 0, 0, 0, 0, 800};
    hh_set_targets(h, 50, t1, PROTO_FLAG_TORQUE);
    hh_run(h, 1); /* last host frame at this step */
    uint32_t last = h->step;
    int hold_at = -1, fault_at = -1;
    for (int i = 0; i < 700; i++) {
        hh_run(h, 1);
        if (hold_at < 0 && g_sim.ctl.safety.mode == MODE_HOLD) hold_at = (int)(h->step - last);
        if (fault_at < 0 && g_sim.ctl.safety.mode == MODE_FAULT) fault_at = (int)(h->step - last);
        if (g_sim.ctl.safety.mode == MODE_HOLD) {
            for (int j = 0; j < 6; j++) TT_ASSERT(g_sim.servo[j].torque); /* torque stays on in HOLD */
        }
    }
    printf("    HOLD after %d ms of silence, FAULT after %d ms\n", hold_at, fault_at);
    TT_EQ(hold_at, 101);
    TT_EQ(fault_at, 501);
    TT_EQ(g_sim.ctl.safety.faults & FAULT_WATCHDOG, FAULT_WATCHDOG);
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
    proto_state_t st;
    hh_last_state(h, &st);
    TT_EQ(st.mode, PROTO_MODE_FAULT);
}

TT_TEST(hold_freezes_command_and_ping_resumes) {
    host_t *h = boot_active();
    static const int16_t far[6] = {2000, 0, 0, 0, 0, 800};
    hh_set_targets(h, 2000, far, PROTO_FLAG_TORQUE); /* slow move */
    run_fed(h, 451);                                  /* last PING at 450 ms into the move */
    int silent = 0;
    while (g_sim.ctl.safety.mode != MODE_HOLD && silent < 200) {
        hh_run(h, 1);
        silent++;
    }
    TT_EQ(silent, 101); /* > 100 ms after the last frame */
    TT_EQ(g_sim.ctl.safety.mode, MODE_HOLD);
    float p = g_sim.ctl.traj.cmd[0].p;
    TT_ASSERT(p > 10 && p < 1990);
    hh_run(h, 50);
    TT_EQ(g_sim.ctl.traj.cmd[0].p, p);
    TT_EQ(g_sim.ctl.traj.cmd[0].v, 0);
    hh_send(h, PROTO_T_PING, NULL, 0);
    hh_run(h, 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    run_fed(h, 200);
    TT_EQ(g_sim.ctl.traj.cmd[0].p, p); /* still holding until new targets arrive */
    TT_NEAR(g_sim.ctl.meas[0].pos_mrad, p, 2.0);
}

TT_TEST(two_step_fault_clear_via_frames) {
    host_t *h = boot_active();
    hh_run(h, 600); /* silent -> WATCHDOG fault */
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    uint8_t s = hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), -PROTO_ERR_FAULTED);
    s = hh_set_targets(h, 100, k_home, 0); /* torque-off request while faulted: accepted */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    s = hh_clear(h, FAULT_WATCHDOG);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT); /* no bits left, but FAULT until SET_TARGETS */
    TT_EQ(g_sim.ctl.safety.faults, 0);
    hh_run(h, 700); /* torque off: watchdog disarmed */
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_EQ(g_sim.ctl.safety.faults, 0);
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
    s = hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 2);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    for (int j = 0; j < 6; j++) TT_ASSERT(g_sim.servo[j].torque);
}

TT_TEST(soft_estop) {
    host_t *h = boot_active();
    uint8_t s = hh_send(h, PROTO_T_ESTOP, NULL, 0);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_ESTOP), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_ESTOP_SOFT);
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque); /* torque-off sent in the same cycle */
    s = hh_clear(h, FAULT_ESTOP_SOFT);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_EQ(g_sim.ctl.safety.faults, 0);
    s = hh_set_targets(h, 100, k_home, 0); /* torque-off SET_TARGETS: FAULT -> IDLE */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
}

TT_TEST(hardware_estop_loop) {
    host_t *h = boot_active();
    g_sim.estop_closed = false; /* loop opens: servo power cut */
    run_fed(h, 50);
    uint16_t f = g_sim.ctl.safety.faults;
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_ASSERT(f & FAULT_ESTOP_HW);
    TT_ASSERT(f & FAULT_BUS_UNDERVOLT);
    TT_ASSERT(f & FAULT_SERVO_COMMS);
    uint8_t s = hh_clear(h, FAULT_DEFINED_MASK);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1); /* ACKed, but the open loop keeps the bit */
    TT_ASSERT(g_sim.ctl.safety.faults & FAULT_ESTOP_HW);
    g_sim.estop_closed = true;
    run_fed(h, 100); /* power back: voltage recovers, servos answer again */
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT); /* still latched */
    s = hh_clear(h, FAULT_DEFINED_MASK);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.faults, 0);
    s = hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE); /* step 2 */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
}

TT_TEST(servo_comms_fault_three_missed_polls) {
    host_t *h = boot_active();
    g_sim.servo[3].silent = true;
    int at = -1;
    for (int i = 0; i < 40; i++) {
        if (i % 20 == 0) hh_send(h, PROTO_T_PING, NULL, 0);
        hh_run(h, 1);
        if (at < 0 && (g_sim.ctl.safety.faults & FAULT_SERVO_COMMS)) at = i + 1;
    }
    printf("    SERVO_COMMS latched %d ms after servo 3 went silent\n", at);
    TT_ASSERT(at > 12 && at <= 19); /* third missed poll of joint 3 (polled every 6 ms) */
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    hh_run(h, 5);
    proto_state_t st;
    hh_last_state(h, &st);
    TT_ASSERT(st.joint[3].status & JS_COMMS_FAULT);
    TT_ASSERT(!(st.joint[3].status & JS_FEEDBACK_OK));
    TT_ASSERT(st.joint[2].status & JS_FEEDBACK_OK);
}

TT_TEST(overcurrent_fault) {
    host_t *h = boot_active();
    g_sim.servo[1].inject_current = true;
    g_sim.servo[1].current_override_ma = 3000;
    run_fed(h, 30);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE); /* filter: needs 8 consecutive reads (~48 ms) */
    run_fed(h, 30);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_OVERCURRENT);
    proto_state_t st;
    hh_run(h, 5);
    hh_last_state(h, &st);
    TT_ASSERT(st.joint[1].status & JS_OVERCURRENT);
    TT_ASSERT(!(st.joint[0].status & JS_OVERCURRENT));
}

TT_TEST(current_limit_config_applies) {
    host_t *h = boot_active();
    uint8_t s = hh_config(h, 5, PROTO_CFG_CURRENT_LIMIT, 100); /* holding current is 170 mA */
    run_fed(h, 80);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), 1);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_OVERCURRENT);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
}

TT_TEST(overtemp_fault) {
    host_t *h = boot_active();
    g_sim.servo[4].temp_c = 71.0f;
    run_fed(h, 10);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_OVERTEMP);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    uint8_t s = hh_clear(h, FAULT_OVERTEMP);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_OVERTEMP); /* still hot: stays set */
    g_sim.servo[4].temp_c = 60.0f;
    hh_run(h, 10);
    s = hh_clear(h, FAULT_OVERTEMP);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.faults, 0);
}

TT_TEST(undervolt_is_live_flag_only) {
    host_t *h = boot_active();
    g_sim.supply_v = 10.2f;
    run_fed(h, 10);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE); /* not latched: no mode change */
    TT_EQ(g_sim.ctl.safety.faults, FAULT_BUS_UNDERVOLT);
    g_sim.supply_v = 12.0f;
    run_fed(h, 60);
    TT_EQ(g_sim.ctl.safety.faults, 0);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
}

TT_TEST(nack_codes) {
    host_t *h = boot();
    uint8_t pl[16] = {0};
    uint8_t s1 = hh_send(h, PROTO_T_SET_TARGETS, pl, 14);
    uint8_t s2 = hh_send(h, 0x07, NULL, 0);
    uint8_t s3 = hh_send(h, PROTO_T_STATE, pl, 0);
    uint8_t s4 = hh_config(h, 0, 0x22, 1);
    uint8_t s5 = hh_config(h, 6, PROTO_CFG_VMAX, 1000);
    uint8_t s6 = hh_config(h, 0, PROTO_CFG_WD_HOLD_MS, 50);
    uint8_t s7 = hh_config(h, 0xFF, PROTO_CFG_WD_HOLD_MS, 600);
    int16_t z[6] = {0, 0, 0, 0, 0, 800};
    uint8_t s8 = hh_set_targets(h, 100, z, 0x03);
    uint8_t s9 = hh_set_targets(h, 0, z, 1);
    uint8_t s10 = hh_set_targets(h, 5001, z, 1);
    uint8_t s11 = hh_stream(h, 999);
    uint8_t s12 = hh_clear(h, 0x0100);
    uint8_t s13 = hh_config(h, 2, PROTO_CFG_DIRECTION, 2);
    uint8_t s14 = hh_send(h, PROTO_T_PING, pl, 1);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s1, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_LEN);
    TT_EQ(hh_reply(h, s2, 0x07), -PROTO_ERR_UNKNOWN_TYPE);
    TT_EQ(hh_reply(h, s3, PROTO_T_STATE), -PROTO_ERR_UNKNOWN_TYPE);
    TT_EQ(hh_reply(h, s4, PROTO_T_CONFIG), -PROTO_ERR_UNKNOWN_KEY);
    TT_EQ(hh_reply(h, s5, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s6, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s7, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s8, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s9, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s10, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s11, PROTO_T_SET_STREAM), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s12, PROTO_T_CLEAR_FAULT), 1); /* undefined bits ignored */
    TT_EQ(hh_reply(h, s13, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    TT_EQ(hh_reply(h, s14, PROTO_T_PING), -PROTO_ERR_BAD_LEN);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
    TT_EQ(g_sim.ctl.stats.nacks, 13);
}

TT_TEST(corrupt_frames_get_no_reply_and_dont_feed_watchdog) {
    host_t *h = boot_active();
    hh_clear_frames(h);
    for (int i = 0; i < 300; i++) {
        uint8_t buf[16];
        size_t n = proto_encode(buf, sizeof buf, PROTO_T_PING, (uint8_t)i, NULL, 0);
        buf[n - 1] ^= 0xFF; /* bad CRC */
        sim_host_send(&g_sim, buf, n);
        hh_run(h, 2);
    }
    TT_EQ(hh_count(h, PROTO_T_PONG), 0);
    TT_EQ(hh_count(h, PROTO_T_NACK), 0);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT); /* watchdog was not fed */
    TT_ASSERT(g_sim.ctl.host_parser.stats.crc_errors >= 300);
}

TT_TEST(set_stream_rates) {
    host_t *h = boot();
    uint8_t s = hh_stream(h, 1000);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_STREAM), 1);
    hh_clear_frames(h);
    hh_run(h, 1000);
    TT_EQ(hh_count(h, PROTO_T_STATE), 1000);
    hh_stream(h, 2500);
    hh_run(h, 1);
    hh_clear_frames(h);
    hh_run(h, 1000);
    TT_EQ(hh_count(h, PROTO_T_STATE), 400);
    hh_stream(h, 0);
    hh_run(h, 1);
    hh_clear_frames(h);
    hh_run(h, 1000);
    TT_EQ(hh_count(h, PROTO_T_STATE), 0);
}

TT_TEST(limit_clamp_flag_and_self_clear) {
    host_t *h = boot_active();
    static const int16_t out_of_range[6] = {0, 2000, 0, 0, 0, 800}; /* shoulder max 750 */
    hh_set_targets(h, 1000, out_of_range, PROTO_FLAG_TORQUE);
    hh_run(h, 10);
    proto_state_t st;
    hh_last_state(h, &st);
    TT_ASSERT(st.fault_flags & FAULT_LIMIT_CLAMP);
    TT_EQ(st.mode, PROTO_MODE_ACTIVE);
    TT_ASSERT(st.joint[1].status & JS_TARGET_CLAMPED);
    TT_ASSERT(!(st.joint[0].status & JS_TARGET_CLAMPED));
    run_fed(h, 1500);
    TT_NEAR(g_sim.ctl.meas[1].pos_mrad, 750, 2);
    /* a too-fast move is stretched, also flagged */
    static const int16_t fast[6] = {2000, 0, 0, 0, 0, 800};
    hh_set_targets(h, 10, fast, PROTO_FLAG_TORQUE);
    hh_run(h, 10);
    hh_last_state(h, &st);
    TT_ASSERT(st.fault_flags & FAULT_LIMIT_CLAMP);
    TT_ASSERT(g_sim.ctl.last_plan.stretched);
    TT_NEAR(g_sim.ctl.last_plan.T, 1.875 * 2000 / 3000, 0.01);
    /* in range and feasible: flag clears */
    hh_set_targets(h, 2000, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 10);
    hh_last_state(h, &st);
    TT_EQ(st.fault_flags & FAULT_LIMIT_CLAMP, 0);
    TT_EQ(st.joint[1].status & JS_TARGET_CLAMPED, 0);
}

TT_TEST(measured_velocity_never_exceeds_vmax) {
    host_t *h = boot_active();
    run_fed(h, 300);
    static const int16_t a[6] = {2500, 1200, -2600, 1800, -2900, 1600};
    static const int16_t b[6] = {-2500, -1700, 2200, -1800, 2900, 0};
    double vmax_ratio = 0;
    armfw_config_t cfg;
    config_defaults(&cfg);
    for (int k = 0; k < 6; k++) {
        hh_set_targets(h, 100, (k % 2) ? b : a, PROTO_FLAG_TORQUE);
        for (int i = 0; i < 700; i++) {
            if (i % 50 == 0) hh_send(h, PROTO_T_PING, NULL, 0);
            hh_run(h, 1);
            for (int j = 0; j < 6; j++) {
                double r = fabs(g_sim.ctl.traj.cmd[j].v) / cfg.joint[j].vmax_mrad_s;
                if (r > vmax_ratio) vmax_ratio = r;
            }
        }
    }
    printf("    peak commanded |v|/vmax over full-range moves: %.4f\n", vmax_ratio);
    TT_ASSERT(vmax_ratio <= 1.0005);
    TT_ASSERT(vmax_ratio > 0.95); /* the moves were vmax-bound */
}

TT_TEST(config_calibration_offset_direction) {
    host_t *h = boot();
    /* calibration keys are refused with torque on */
    hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE);
    uint8_t s = hh_config(h, 0, PROTO_CFG_OFFSET_TICKS, 100);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    hh_set_targets(h, 100, k_home, 0);
    hh_run(h, 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
    s = hh_config(h, 0, PROTO_CFG_OFFSET_TICKS, 100);
    uint8_t s2 = hh_config(h, 2, PROTO_CFG_DIRECTION, -1);
    hh_run(h, 20);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), 1);
    TT_EQ(hh_reply(h, s2, PROTO_T_CONFIG), 1);
    /* servo 0 sits at tick 2048 -> joint = -100 ticks */
    TT_NEAR(g_sim.ctl.meas[0].pos_mrad, -100 * CFG_MRAD_PER_TICK, 0.01);
    /* direction -1 on joint 2: positive joint target -> servo goal below center */
    static const int16_t t[6] = {0, 0, 500, 0, 0, 800};
    hh_set_targets(h, 200, t, PROTO_FLAG_TORQUE);
    run_fed(h, 400);
    TT_NEAR(g_sim.servo[2].goal_ticks, 2048 - 500 * CFG_TICKS_PER_MRAD, 1.0);
    TT_NEAR(g_sim.ctl.meas[2].pos_mrad, 500, 2);
    TT_NEAR(g_sim.servo[0].goal_ticks, 2148, 0.5);
}

TT_TEST(torque_off_via_set_targets) {
    host_t *h = boot_active();
    uint8_t s = hh_set_targets(h, 100, k_home, 0);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
    hh_run(h, 1000); /* idle: no watchdog fault */
    TT_EQ(g_sim.ctl.safety.faults, 0);
}

TT_TEST(half_duplex_echo_tolerated) {
    sim_init(&g_sim, 0);
    g_sim.echo_tx = true;
    hh_init(&g_host, &g_sim);
    host_t *h = &g_host;
    hh_run(h, 20);
    static const int16_t t[6] = {400, -300, 200, -100, 600, 1000};
    hh_set_targets(h, 300, t, PROTO_FLAG_TORQUE);
    run_fed(h, 1000);
    TT_EQ(g_sim.ctl.safety.faults, 0);
    for (int j = 0; j < 6; j++) TT_NEAR(g_sim.ctl.meas[j].pos_mrad, t[j], 2);
    TT_EQ(g_sim.ctl.stats.servo_misses, 0);
}

TT_TEST(jitter_and_overrun_reporting) {
    sim_init(&g_sim, 0);
    g_sim.jitter_us = 40;
    hh_init(&g_host, &g_sim);
    host_t *h = &g_host;
    hh_run(h, 500);
    proto_state_t st;
    hh_last_state(h, &st);
    TT_ASSERT(st.loop_jitter_max_us > 0 && st.loop_jitter_max_us <= 40);
    TT_EQ(st.loop_overruns, 0);
    /* a late tick (2.5 ms gap) counts as an overrun and shows as jitter */
    g_sim.jitter_us = 0;
    g_sim.now_us += 1500;
    hh_run(h, 10);
    hh_last_state(h, &st);
    TT_EQ(st.loop_overruns, 1);
    hh_run(h, 10);
    hh_last_state(h, &st);
    TT_ASSERT(st.loop_jitter_max_us == 0); /* reset after each STATE */
}

TT_TEST(servo_bus_traffic_per_cycle) {
    host_t *h = boot_active();
    g_sim.max_tx_bytes = 0;
    run_fed(h, 500);
    /* goal SYNC_WRITE (26) + READ (8) normally, + torque refresh (20) every 100 ms */
    TT_EQ(g_sim.last_tx_bytes == 34 || g_sim.last_tx_bytes == 54, 1);
    TT_EQ(g_sim.max_tx_bytes, 54);
    uint32_t bus_us = sb_cycle_bus_us(g_sim.max_tx_bytes, 21, 50);
    TT_ASSERT(bus_us < 1000);
    for (int j = 0; j < 6; j++) TT_ASSERT(g_sim.servo[j].reads_answered >= 80);
}

TT_TEST(torque_off_is_never_refused) {
    host_t *h = boot_active();
    uint8_t s = hh_set_targets(h, 0, k_home, 0x80); /* bad move_ms and a reserved flag bit */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_VALUE);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE); /* but torque went off */
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
    hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 1);
    s = hh_set_targets(h, 0, k_home, 0); /* bad move_ms alone: NACK 2, torque still off */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), -PROTO_ERR_BAD_VALUE);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
    hh_set_targets(h, 100, k_home, PROTO_FLAG_TORQUE);
    hh_run(h, 1);
    s = hh_set_targets(h, 100, k_home, 0);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_SET_TARGETS), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
}

TT_MAIN()
