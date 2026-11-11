/* End to end through the protocol: CONFIG 0x20 (save to flash) / 0x21 (restore defaults), values
 * surviving a reboot of the simulated board, power cuts during the save, and the ADC path feeding
 * STATE and the safety rules from the simulated rail. */
#include "host_helpers.h"

static sim_t g_sim;
static host_t g_host;

#define KEY_SAVE CFG_KEY_SAVE_FLASH
#define KEY_DEFAULTS CFG_KEY_RESTORE_DEFAULTS
#define MAGIC CFG_COMMIT_MAGIC

static host_t *boot(void) {
    sim_init(&g_sim, 0);
    hh_init(&g_host, &g_sim);
    hh_run(&g_host, 20);
    return &g_host;
}

static host_t *reboot(void) {
    sim_reboot(&g_sim, 0);
    hh_init(&g_host, &g_sim);
    hh_run(&g_host, 20);
    return &g_host;
}

/* Run until the reply to seq arrives (max ms). Returns the reply code (hh_reply) and the steps. */
static int wait_reply(host_t *h, uint8_t seq, int max_ms, int *steps) {
    for (int i = 0; i < max_ms; i++) {
        int r = hh_reply(h, seq, PROTO_T_CONFIG);
        if (r) {
            if (steps) *steps = i;
            return r;
        }
        hh_run(h, 1);
    }
    return 0;
}

static void set_cal(host_t *h) {
    for (int j = 0; j < 6; j++) hh_config(h, (uint8_t)j, PROTO_CFG_OFFSET_TICKS, 10 * (j + 1) - 30);
    hh_config(h, 2, PROTO_CFG_DIRECTION, -1);
    hh_config(h, 5, PROTO_CFG_CURRENT_LIMIT, 1000);
    hh_config(h, 1, PROTO_CFG_LIMIT_MAX, 700);
    hh_config(h, 0xFF, PROTO_CFG_WD_OFF_MS, 800);
    hh_run(h, 2);
}

static void check_cal(const armfw_config_t *c) {
    for (int j = 0; j < 6; j++) TT_EQ(c->joint[j].offset_ticks, 10 * (j + 1) - 30);
    TT_EQ(c->joint[2].direction, -1);
    TT_EQ(c->joint[0].direction, 1);
    TT_EQ(c->joint[5].current_limit_ma, 1000);
    TT_EQ(c->joint[1].limit_max_mrad, 700);
    TT_EQ(c->wd_off_ms, 800);
    TT_EQ(c->wd_hold_ms, 100);
}

TT_TEST(save_survives_reboot) {
    host_t *h = boot();
    TT_ASSERT(!g_sim.ctl.cal_from_flash);
    set_cal(h);
    uint8_t s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), 0); /* deferred: ACK only once written and verified */
    int steps = 0;
    TT_EQ(wait_reply(h, s, 100, &steps), 1);
    TT_ASSERT_MSG(steps >= 10 && steps <= 20, "ACK after %d ms", steps);
    TT_EQ(g_sim.ctl.stats.cal_saves, 1);
    /* the board keeps running normally afterwards */
    hh_clear_frames(h);
    hh_run(h, 100);
    TT_EQ(hh_count(h, PROTO_T_STATE), 20);

    h = reboot();
    TT_ASSERT(g_sim.ctl.cal_from_flash);
    check_cal(&g_sim.ctl.cfg);
    /* the restored offsets are in effect: servo at tick 2048 reads -(offset) ticks */
    proto_state_t st;
    TT_ASSERT(hh_last_state(h, &st));
    TT_NEAR(st.joint[0].pos_mrad, 20 * 1.5339808, 2); /* offset -20 ticks */
    TT_NEAR(st.joint[2].pos_mrad, 0, 2);              /* offset 0, direction -1 */
}

TT_TEST(restore_defaults_ram_only) {
    host_t *h = boot();
    set_cal(h);
    uint8_t s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    TT_EQ(wait_reply(h, s, 100, NULL), 1);
    s = hh_config(h, 0xFF, KEY_DEFAULTS, MAGIC);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), 1);
    armfw_config_t d;
    config_defaults(&d);
    TT_EQ(g_sim.ctl.cfg.joint[3].offset_ticks, 0);
    TT_EQ(g_sim.ctl.cfg.wd_off_ms, d.wd_off_ms);
    TT_EQ(g_sim.ctl.cfg.joint[1].limit_max_mrad, d.joint[1].limit_max_mrad);
    /* not persisted until saved */
    reboot();
    check_cal(&g_sim.ctl.cfg);
    /* defaults + save = defaults after reboot */
    h = &g_host;
    s = hh_config(h, 0xFF, KEY_DEFAULTS, MAGIC);
    hh_run(h, 1);
    s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    TT_EQ(wait_reply(h, s, 100, NULL), 1);
    reboot();
    TT_ASSERT(g_sim.ctl.cal_from_flash);
    TT_EQ(g_sim.ctl.cfg.joint[0].offset_ticks, 0);
    TT_EQ(g_sim.ctl.cfg.wd_off_ms, 500);
}

TT_TEST(commit_keys_validation) {
    host_t *h = boot();
    uint8_t s;
    s = hh_config(h, 0xFF, KEY_SAVE, 0x5AFF); /* wrong magic */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    s = hh_config(h, 0, KEY_SAVE, MAGIC); /* global key on a joint */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    s = hh_config(h, 0xFF, KEY_DEFAULTS, 0);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_BAD_VALUE);
    s = hh_config(h, 0xFF, 0x22, MAGIC); /* still unknown */
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_UNKNOWN_KEY);
    /* a second save while the first is still running is refused with NACK 4; the first one ACKs */
    uint8_t s1 = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    uint8_t s2 = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s2, PROTO_T_CONFIG), -PROTO_ERR_FAULTED);
    TT_EQ(wait_reply(h, s1, 100, NULL), 1);
    /* restore defaults while torque is on: refused like the calibration keys */
    static const int16_t home[6] = {0, 0, 0, 0, 0, 800};
    hh_set_targets(h, 200, home, PROTO_FLAG_TORQUE);
    hh_run(h, 2);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    s = hh_config(h, 0xFF, KEY_DEFAULTS, MAGIC);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -CFG_ERR_TORQUE_ON);
    /* saving while torque is on is fine (flash work is incremental, the loop keeps its period) */
    s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    TT_EQ(wait_reply(h, s, 100, NULL), 1);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    TT_EQ(g_sim.ctl.overruns, 0);
    /* no flash at all: NACK 4 */
    g_sim.ctl.store.fp = NULL;
    s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CONFIG), -PROTO_ERR_FAULTED);
}

TT_TEST(power_cut_during_save_keeps_previous) {
    for (int32_t budget = 0; budget < 110; budget += 3) {
        host_t *h = boot();
        set_cal(h);
        uint8_t s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
        TT_EQ(wait_reply(h, s, 100, NULL), 1);
        /* change a value and save again, but the power dies part way */
        hh_config(h, 4, PROTO_CFG_VMAX, 1234);
        hh_run(h, 1);
        g_sim.flash.budget = budget;
        g_sim.flash.torn_mode = (ff_torn_mode_t)(budget % FF_TORN_MODES);
        s = hh_config(h, 0xFF, KEY_SAVE, MAGIC);
        int r = 0;
        for (int i = 0; i < 100 && !g_sim.flash.dead && !(r = hh_reply(h, s, PROTO_T_CONFIG)); i++) hh_run(h, 1);
        reboot();
        TT_ASSERT(g_sim.ctl.cal_from_flash);
        check_cal(&g_sim.ctl.cfg); /* everything else intact either way */
        int16_t vmax = g_sim.ctl.cfg.joint[4].vmax_mrad_s;
        TT_ASSERT_MSG(vmax == 4000 || vmax == 1234, "budget %d vmax %d", budget, vmax);
        if (r == 1) TT_EQ(vmax, 1234);
    }
}

TT_TEST(state_reports_adc_values) {
    host_t *h = boot();
    g_sim.supply_v = 11.4f;
    hh_run(h, 50);
    proto_state_t st;
    TT_ASSERT(hh_last_state(h, &st));
    TT_NEAR(st.bus_mv, sim_bus_mv(&g_sim), 15);
    TT_NEAR(st.bus_ma, sim_bus_ma(&g_sim), 12);
    TT_NEAR(g_sim.ctl.analog.vin_mv, 11400, 15);
    TT_NEAR(g_sim.ctl.analog.board_temp_dc, 300, 3);
    TT_ASSERT(g_sim.ctl.analog.ntc_ok);
}

TT_TEST(bus_overcurrent_from_adc_codes) {
    host_t *h = boot();
    static const int16_t home[6] = {0, 0, 0, 0, 0, 800};
    hh_set_targets(h, 200, home, PROTO_FLAG_TORQUE);
    hh_run(h, 5);
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    /* 9 A on the shunt as raw INA240 codes, with healthy servos (their own currents are small) */
    g_sim.adc_override = true;
    sim_adc_codes(&g_sim, &g_sim.adc_raw);
    analog_params_t p;
    analog_default_params(&p);
    g_sim.adc_raw.isense = analog_code_for_bus_ma(&p, 9000, 3300);
    int n = 0;
    while (g_sim.ctl.safety.mode == MODE_ACTIVE && n < 200) {
        if (n % 40 == 0) hh_send(h, PROTO_T_PING, NULL, 0);
        hh_run(h, 1);
        n++;
    }
    TT_ASSERT_MSG(n >= SAFETY_BUS_OC_MS && n <= SAFETY_BUS_OC_MS + 4, "tripped after %d ms", n);
    TT_ASSERT(g_sim.ctl.safety.faults & FAULT_OVERCURRENT);
    TT_EQ(g_sim.ctl.safety.joint_overcurrent, 0); /* it was the bus, not a joint */
    for (int j = 0; j < 6; j++) TT_ASSERT(!g_sim.servo[j].torque);
}

TT_TEST(dead_adc_flags_undervolt_only) {
    host_t *h = boot();
    g_sim.adc_dead = true;
    hh_run(h, 30);
    TT_EQ(g_sim.ctl.safety.faults, FAULT_BUS_UNDERVOLT);
    TT_EQ(g_sim.ctl.safety.mode, MODE_IDLE);
    proto_state_t st;
    TT_ASSERT(hh_last_state(h, &st));
    TT_EQ(st.bus_mv, 0);
    g_sim.adc_dead = false;
    hh_run(h, 60);
    TT_EQ(g_sim.ctl.safety.faults, 0);
}

TT_TEST(board_overtemp_latches) {
    host_t *h = boot();
    g_sim.board_temp_c = 95.0f;
    hh_run(h, 1500);
    TT_ASSERT(g_sim.ctl.safety.faults & FAULT_OVERTEMP);
    TT_EQ(g_sim.ctl.safety.mode, MODE_FAULT);
    TT_EQ(g_sim.ctl.safety.joint_overtemp, 0);
    g_sim.board_temp_c = 40.0f;
    hh_run(h, 1000);
    uint8_t s = hh_clear(h, FAULT_OVERTEMP);
    hh_run(h, 1);
    TT_EQ(hh_reply(h, s, PROTO_T_CLEAR_FAULT), 1);
    TT_EQ(g_sim.ctl.safety.faults, 0);
}

TT_MAIN()
