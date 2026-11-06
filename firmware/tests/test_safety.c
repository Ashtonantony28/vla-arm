/* Safety state machine: every transition and fault rule in spec/protocol.md. */
#include "armfw/safety.h"
#include "tt.h"

typedef struct {
    safety_t s;
    armfw_config_t cfg;
    safety_inputs_t in;
} fx_t;

static void fx_init(fx_t *f, uint32_t t0) {
    config_defaults(&f->cfg);
    memset(&f->in, 0, sizeof f->in);
    f->in.now_us = t0;
    f->in.estop_loop_closed = true;
    f->in.bus_mv = 12000;
    for (int j = 0; j < 6; j++) f->in.temp_c[j] = 35;
    safety_init(&f->s, t0);
}

/* advance n ms, one update per ms */
static void run_ms(fx_t *f, int n) {
    for (int i = 0; i < n; i++) {
        f->in.now_us += 1000;
        safety_update(&f->s, &f->cfg, &f->in);
    }
}

static void fresh_all(fx_t *f, bool v) {
    for (int j = 0; j < 6; j++) f->in.fresh[j] = v;
}

TT_TEST(starts_idle_and_torque_toggles) {
    fx_t f;
    fx_init(&f, 0);
    TT_EQ(f.s.mode, MODE_IDLE);
    TT_EQ(f.s.faults, 0);
    TT_EQ(safety_request_torque(&f.s, true, f.in.now_us), 0);
    TT_EQ(f.s.mode, MODE_ACTIVE);
    TT_EQ(safety_request_torque(&f.s, false, f.in.now_us), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(watchdog_hold_at_100ms_fault_at_500ms) {
    fx_t f;
    fx_init(&f, 1000);
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 100); /* silent exactly 100 ms: not yet > 100 */
    TT_EQ(f.s.mode, MODE_ACTIVE);
    run_ms(&f, 1); /* 101 ms */
    TT_EQ(f.s.mode, MODE_HOLD);
    TT_EQ(f.s.faults, 0);
    run_ms(&f, 399); /* 500 ms */
    TT_EQ(f.s.mode, MODE_HOLD);
    run_ms(&f, 1); /* 501 ms */
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(f.s.faults, FAULT_WATCHDOG);
    /* latched: host frames alone do not leave FAULT */
    safety_host_frame(&f.s, f.in.now_us);
    run_ms(&f, 10);
    TT_EQ(f.s.mode, MODE_FAULT);
}

TT_TEST(host_frame_in_hold_returns_to_active) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 150);
    TT_EQ(f.s.mode, MODE_HOLD);
    safety_host_frame(&f.s, f.in.now_us);
    TT_EQ(f.s.mode, MODE_ACTIVE);
    run_ms(&f, 100);
    TT_EQ(f.s.mode, MODE_ACTIVE);
    run_ms(&f, 1);
    TT_EQ(f.s.mode, MODE_HOLD);
}

TT_TEST(regular_frames_keep_active) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    for (int i = 0; i < 100; i++) {
        run_ms(&f, 99);
        safety_host_frame(&f.s, f.in.now_us);
    }
    TT_EQ(f.s.mode, MODE_ACTIVE);
    TT_EQ(f.s.faults, 0);
}

TT_TEST(watchdog_disarmed_in_idle) {
    fx_t f;
    fx_init(&f, 0);
    run_ms(&f, 10000);
    TT_EQ(f.s.mode, MODE_IDLE);
    TT_EQ(f.s.faults, 0);
    /* torque-on after a long silence starts a fresh watchdog period */
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 50);
    TT_EQ(f.s.mode, MODE_ACTIVE);
}

TT_TEST(watchdog_configurable) {
    fx_t f;
    fx_init(&f, 0);
    f.cfg.wd_hold_ms = 20;
    f.cfg.wd_off_ms = 40;
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 21);
    TT_EQ(f.s.mode, MODE_HOLD);
    run_ms(&f, 20);
    TT_EQ(f.s.mode, MODE_FAULT);
}

TT_TEST(watchdog_time_wraparound) {
    fx_t f;
    fx_init(&f, 0xFFFFFFFFu - 50000u);
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 100);
    TT_EQ(f.s.mode, MODE_ACTIVE);
    run_ms(&f, 1);
    TT_EQ(f.s.mode, MODE_HOLD);
    run_ms(&f, 400);
    TT_EQ(f.s.mode, MODE_FAULT);
}

TT_TEST(estop_soft_two_step_clear) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    safety_estop_soft(&f.s);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(f.s.faults, FAULT_ESTOP_SOFT);
    TT_EQ(safety_request_torque(&f.s, true, f.in.now_us), PROTO_ERR_FAULTED);
    TT_EQ(f.s.mode, MODE_FAULT);
    /* torque-off while faulted is accepted and changes nothing */
    TT_EQ(safety_request_torque(&f.s, false, f.in.now_us), 0);
    TT_EQ(f.s.mode, MODE_FAULT);
    /* step 1: clear -> IDLE, torque still off */
    TT_EQ(safety_clear(&f.s, FAULT_ESTOP_SOFT), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
    run_ms(&f, 10);
    TT_EQ(f.s.mode, MODE_IDLE);
    /* step 2: torque enable */
    TT_EQ(safety_request_torque(&f.s, true, f.in.now_us), 0);
    TT_EQ(f.s.mode, MODE_ACTIVE);
}

TT_TEST(clear_needs_every_latched_bit) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    safety_estop_soft(&f.s);
    run_ms(&f, 600); /* watchdog disarmed in FAULT, so no WATCHDOG bit */
    TT_EQ(f.s.faults, FAULT_ESTOP_SOFT);
    f.in.temp_c[2] = 75;
    f.in.fresh[2] = true;
    run_ms(&f, 1);
    f.in.temp_c[2] = 40;
    run_ms(&f, 1);
    f.in.fresh[2] = false;
    TT_EQ(f.s.faults, FAULT_ESTOP_SOFT | FAULT_OVERTEMP);
    TT_EQ(safety_clear(&f.s, FAULT_ESTOP_SOFT), 0);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(safety_request_torque(&f.s, true, f.in.now_us), PROTO_ERR_FAULTED);
    TT_EQ(safety_clear(&f.s, FAULT_OVERTEMP), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
    TT_EQ(safety_request_torque(&f.s, true, f.in.now_us), 0);
}

TT_TEST(estop_hw_clears_only_when_loop_closed) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    f.in.estop_loop_closed = false;
    run_ms(&f, 1);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(f.s.faults & FAULT_ESTOP_HW, FAULT_ESTOP_HW);
    TT_EQ(safety_clear(&f.s, FAULT_ESTOP_HW), PROTO_ERR_FAULTED);
    TT_EQ(f.s.mode, MODE_FAULT);
    f.in.estop_loop_closed = true;
    run_ms(&f, 5);
    /* closing the loop alone does not clear it */
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(safety_clear(&f.s, FAULT_ESTOP_HW), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(overcurrent_filtered_and_latched) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    f.in.current_ma[1] = 2500; /* limit 2000 */
    for (int i = 0; i < SAFETY_OC_SAMPLES - 1; i++) {
        safety_host_frame(&f.s, f.in.now_us);
        f.in.fresh[1] = true;
        run_ms(&f, 1);
        f.in.fresh[1] = false;
        run_ms(&f, 5); /* non-fresh cycles neither count nor reset */
    }
    TT_EQ(f.s.mode, MODE_ACTIVE);
    /* a sample under the limit resets the filter */
    f.in.current_ma[1] = 100;
    f.in.fresh[1] = true;
    run_ms(&f, 1);
    TT_EQ(f.s.oc_count[1], 0);
    f.in.current_ma[1] = -2500; /* sign does not matter */
    for (int i = 0; i < SAFETY_OC_SAMPLES; i++) {
        safety_host_frame(&f.s, f.in.now_us);
        run_ms(&f, 1);
    }
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(f.s.faults, FAULT_OVERCURRENT);
    TT_EQ(safety_clear(&f.s, FAULT_OVERCURRENT), PROTO_ERR_FAULTED); /* still over */
    f.in.current_ma[1] = 50;
    run_ms(&f, 1);
    TT_EQ(safety_clear(&f.s, FAULT_OVERCURRENT), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(per_joint_current_limit_config) {
    fx_t f;
    fx_init(&f, 0);
    f.cfg.joint[5].current_limit_ma = 500;
    f.in.current_ma[5] = 600;
    f.in.current_ma[0] = 1900;
    fresh_all(&f, true);
    run_ms(&f, SAFETY_OC_SAMPLES);
    TT_EQ(f.s.faults, FAULT_OVERCURRENT);
    TT_EQ(f.s.joint_overcurrent, 1 << 5);
}

TT_TEST(bus_overcurrent) {
    fx_t f;
    fx_init(&f, 0);
    f.in.bus_ma = 8001;
    run_ms(&f, SAFETY_BUS_OC_MS - 1);
    TT_EQ(f.s.faults, 0);
    run_ms(&f, 1);
    TT_EQ(f.s.faults, FAULT_OVERCURRENT);
    TT_EQ(f.s.mode, MODE_FAULT);
}

TT_TEST(overtemp_threshold) {
    fx_t f;
    fx_init(&f, 0);
    f.in.temp_c[4] = 70;
    fresh_all(&f, true);
    run_ms(&f, 10);
    TT_EQ(f.s.faults, 0);
    f.in.temp_c[4] = 71;
    run_ms(&f, 1);
    TT_EQ(f.s.faults, FAULT_OVERTEMP);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(safety_clear(&f.s, FAULT_OVERTEMP), PROTO_ERR_FAULTED);
    f.in.temp_c[4] = 69;
    run_ms(&f, 1);
    TT_EQ(safety_clear(&f.s, FAULT_OVERTEMP), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(servo_comms_three_misses) {
    fx_t f;
    fx_init(&f, 0);
    f.in.comms_miss[3] = 2;
    run_ms(&f, 5);
    TT_EQ(f.s.faults, 0);
    f.in.comms_miss[3] = 3;
    run_ms(&f, 1);
    TT_EQ(f.s.faults, FAULT_SERVO_COMMS);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(safety_clear(&f.s, FAULT_SERVO_COMMS), PROTO_ERR_FAULTED);
    f.in.comms_miss[3] = 0;
    run_ms(&f, 1);
    TT_EQ(f.s.faults, FAULT_SERVO_COMMS); /* latched */
    TT_EQ(safety_clear(&f.s, FAULT_SERVO_COMMS), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(undervolt_live_with_hysteresis) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    f.in.bus_mv = 10499;
    for (int i = 0; i < SAFETY_UNDERVOLT_SET_MS - 1; i++) {
        safety_host_frame(&f.s, f.in.now_us);
        run_ms(&f, 1);
    }
    TT_EQ(f.s.mode, MODE_ACTIVE); /* 4 ms sag tolerated */
    run_ms(&f, 1);
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(f.s.faults, FAULT_BUS_UNDERVOLT);
    TT_EQ(safety_clear(&f.s, FAULT_BUS_UNDERVOLT), PROTO_ERR_FAULTED);
    f.in.bus_mv = 10600; /* inside the hysteresis band */
    run_ms(&f, 200);
    TT_EQ(f.s.mode, MODE_FAULT);
    f.in.bus_mv = 11500;
    run_ms(&f, SAFETY_UNDERVOLT_CLEAR_MS - 1);
    TT_EQ(f.s.mode, MODE_FAULT);
    run_ms(&f, 1);
    /* self-clears, but torque stays off until a new torque-enable */
    TT_EQ(f.s.faults, 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(limit_clamp_is_informational) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    safety_set_limit_clamp(&f.s, true);
    safety_host_frame(&f.s, f.in.now_us);
    run_ms(&f, 5);
    TT_EQ(f.s.mode, MODE_ACTIVE);
    TT_EQ(f.s.faults, FAULT_LIMIT_CLAMP);
    safety_set_limit_clamp(&f.s, false);
    TT_EQ(f.s.faults, 0);
    safety_set_limit_clamp(&f.s, true);
    TT_EQ(safety_clear(&f.s, FAULT_LIMIT_CLAMP), 0);
    TT_EQ(f.s.faults, 0);
}

TT_TEST(clear_rejects_undefined_bits) {
    fx_t f;
    fx_init(&f, 0);
    safety_estop_soft(&f.s);
    TT_EQ(safety_clear(&f.s, 0x0101), PROTO_ERR_BAD_VALUE);
    TT_EQ(f.s.faults, FAULT_ESTOP_SOFT); /* nothing cleared */
    TT_EQ(safety_clear(&f.s, 0), 0);     /* empty mask: no-op */
    TT_EQ(f.s.mode, MODE_FAULT);
    TT_EQ(safety_clear(&f.s, 0x00FF), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
}

TT_TEST(torque_off_from_hold) {
    fx_t f;
    fx_init(&f, 0);
    safety_request_torque(&f.s, true, f.in.now_us);
    run_ms(&f, 120);
    TT_EQ(f.s.mode, MODE_HOLD);
    TT_EQ(safety_request_torque(&f.s, false, f.in.now_us), 0);
    TT_EQ(f.s.mode, MODE_IDLE);
    run_ms(&f, 1000);
    TT_EQ(f.s.mode, MODE_IDLE);
    TT_EQ(f.s.faults, 0);
}

TT_TEST(faults_latch_from_idle_too) {
    fx_t f;
    fx_init(&f, 0);
    f.in.estop_loop_closed = false;
    run_ms(&f, 1);
    TT_EQ(f.s.mode, MODE_FAULT);
}

TT_MAIN()
