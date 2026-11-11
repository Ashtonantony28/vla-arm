/* Calibration store: record format, A/B page log, wear levelling and power-loss safety against the
 * fake flash (torn programs at every byte offset, torn erases at every double word, ECC errors,
 * program errors, silent corruption, random multi-fault stress). */
#include "armfw/calstore.h"
#include "fake_flash.h"
#include "tt.h"

static fake_flash_t g_ff;
static flash_port_t g_port;

static void ff_reset(void) {
    fake_flash_init(&g_ff);
    fake_flash_port(&g_ff, &g_port);
}

/* A configuration that is distinguishable per index k (and valid). */
static void cfg_k(armfw_config_t *c, int k) {
    config_defaults(c);
    for (int j = 0; j < 6; j++) {
        c->joint[j].offset_ticks = (int32_t)((k * 7 + j * 13) % 8191) - 4095;
        c->joint[j].direction = (int8_t)(((k + j) & 1) ? -1 : 1);
        c->joint[j].vmax_mrad_s = (int16_t)(1 + (k * 31 + j) % 30000);
        c->joint[j].current_limit_ma = (int16_t)(100 + (k * 17 + j) % 3000);
    }
    c->wd_hold_ms = (uint16_t)(50 + k % 100);
    c->wd_off_ms = (uint16_t)(c->wd_hold_ms + 1 + k % 1000);
}

static bool cfg_eq(const armfw_config_t *a, const armfw_config_t *b) {
    for (int j = 0; j < 6; j++) {
        const joint_cfg_t *x = &a->joint[j], *y = &b->joint[j];
        if (x->offset_ticks != y->offset_ticks || x->direction != y->direction ||
            x->limit_min_mrad != y->limit_min_mrad || x->limit_max_mrad != y->limit_max_mrad ||
            x->vmax_mrad_s != y->vmax_mrad_s || x->current_limit_ma != y->current_limit_ma ||
            x->servo_id != y->servo_id)
            return false;
    }
    return a->wd_hold_ms == b->wd_hold_ms && a->wd_off_ms == b->wd_off_ms;
}

/* Run a save until it reports. Returns 1 ok, -1 failed, 0 = flash died (power cut) first. */
static int run_save(calstore_t *cs, const armfw_config_t *c) {
    if (calstore_save_start(cs, c) != 0) return -1;
    for (int i = 0; i < 100000; i++) {
        int r = calstore_service(cs);
        if (g_ff.dead) return 0; /* power is gone: the CPU never sees the outcome */
        if (r) return r;
    }
    return -1;
}

static bool reboot_load(calstore_t *cs, armfw_config_t *out) {
    fake_flash_power_on(&g_ff);
    return calstore_init(cs, &g_port, out);
}

TT_TEST(crc32_check_value) {
    TT_EQ(calstore_crc32((const uint8_t *)"123456789", 9), 0xCBF43926u);
    TT_EQ(calstore_crc32(NULL, 0), 0);
}

TT_TEST(record_roundtrip_and_every_bitflip_rejected) {
    armfw_config_t a, b;
    cfg_k(&a, 5);
    uint8_t rec[CALSTORE_SLOT_LEN];
    calstore_encode(&a, 1234, rec);
    TT_EQ(armfw_get_u32(rec), CALSTORE_MAGIC);
    uint32_t seq = 0;
    TT_ASSERT(calstore_decode(rec, &b, &seq));
    TT_EQ(seq, 1234);
    TT_ASSERT(cfg_eq(&a, &b));
    for (int bit = 0; bit < (int)CALSTORE_SLOT_LEN * 8; bit++) {
        uint8_t r2[CALSTORE_SLOT_LEN];
        memcpy(r2, rec, sizeof r2);
        r2[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        TT_ASSERT_MSG(!calstore_decode(r2, &b, &seq), "bit %d flip accepted", bit);
    }
    /* a CRC-valid record with an out-of-range field is still rejected */
    armfw_config_t bad = a;
    bad.joint[2].direction = 0;
    calstore_encode(&bad, 1, rec);
    TT_ASSERT(!calstore_decode(rec, &b, &seq));
    bad = a;
    bad.wd_off_ms = bad.wd_hold_ms;
    calstore_encode(&bad, 1, rec);
    TT_ASSERT(!calstore_decode(rec, &b, &seq));
    /* other format version */
    calstore_encode(&a, 1, rec);
    armfw_put_u16(&rec[4], 2);
    armfw_put_u32(&rec[CALSTORE_CRC_OFF], calstore_crc32(rec, CALSTORE_CRC_OFF));
    TT_ASSERT(!calstore_decode(rec, &b, &seq));
}

TT_TEST(blank_or_absent_flash_gives_defaults) {
    calstore_t cs;
    armfw_config_t c, d;
    config_defaults(&d);
    ff_reset();
    TT_ASSERT(!calstore_init(&cs, &g_port, &c));
    TT_ASSERT(cfg_eq(&c, &d));
    TT_EQ(cs.active_page, -1);
    TT_ASSERT(!calstore_init(&cs, NULL, &c));
    TT_ASSERT(cfg_eq(&c, &d));
    TT_EQ(calstore_save_start(&cs, &c), -2);
    /* garbage everywhere: defaults, and the next save still works */
    tt_rng_t r = {9};
    for (unsigned p = 0; p < FF_PAGES; p++)
        for (unsigned i = 0; i < FF_PAGE_SIZE; i++) g_ff.mem[p][i] = (uint8_t)tt_rand(&r);
    TT_ASSERT(!calstore_init(&cs, &g_port, &c));
    TT_EQ(cs.garbage_slots, 2 * (FF_PAGE_SIZE / CALSTORE_SLOT_LEN));
    armfw_config_t k;
    cfg_k(&k, 1);
    TT_EQ(run_save(&cs, &k), 1);
    TT_ASSERT(reboot_load(&cs, &c));
    TT_ASSERT(cfg_eq(&c, &k));
}

TT_TEST(save_load_and_wear_levelling) {
    calstore_t cs;
    armfw_config_t c, k;
    ff_reset();
    calstore_init(&cs, &g_port, &c);
    const int N = 1000;
    const uint32_t slots = FF_PAGE_SIZE / CALSTORE_SLOT_LEN;
    TT_EQ(slots, 21);
    for (int i = 1; i <= N; i++) {
        cfg_k(&k, i);
        TT_EQ(run_save(&cs, &k), 1);
        if (i % 97 == 0) { /* reboot now and then: the log resumes where it was */
            TT_ASSERT(reboot_load(&cs, &c));
            TT_ASSERT(cfg_eq(&c, &k));
            TT_EQ(cs.loaded_seq, (uint32_t)i);
        }
    }
    TT_ASSERT(reboot_load(&cs, &c));
    TT_ASSERT(cfg_eq(&c, &k));
    /* one erase per 21 saves, alternating pages */
    uint32_t total = g_ff.erases[0] + g_ff.erases[1];
    TT_NEAR(total, (N + slots - 1) / slots, 1);
    TT_NEAR(g_ff.erases[0], g_ff.erases[1], 1);
    TT_EQ(cs.burned_slots, 0);
    TT_EQ(g_ff.programs, (uint32_t)N * (CALSTORE_SLOT_LEN / 8));
}

TT_TEST(busy_latency_is_polled) {
    calstore_t cs;
    armfw_config_t c, k;
    ff_reset();
    g_ff.busy_polls = 7;
    calstore_init(&cs, &g_port, &c);
    cfg_k(&k, 3);
    TT_EQ(calstore_save_start(&cs, &k), 0);
    TT_EQ(calstore_save_start(&cs, &k), -1); /* busy */
    int steps = 0, r = 0;
    while (!(r = calstore_service(&cs))) steps++;
    TT_EQ(r, 1);
    /* erase + 12 programs, each 7 busy polls + 1 completing poll, plus the verify step */
    TT_ASSERT_MSG(steps >= 13 * 7 && steps <= 13 * 8 + 3, "steps %d", steps);
    TT_ASSERT(reboot_load(&cs, &c));
    TT_ASSERT(cfg_eq(&c, &k));
    /* a flash that never finishes: the save gives up instead of hanging */
    g_ff.busy_polls = 100000;
    cfg_k(&k, 4);
    armfw_config_t old = c;
    TT_EQ(run_save(&cs, &k), -1);
    g_ff.busy_polls = 0;
    TT_ASSERT(reboot_load(&cs, &c));
    TT_ASSERT(cfg_eq(&c, &old));
}

/* Lay down `n` successful saves (configs 1..n) on blank flash. */
static void prepare(calstore_t *cs, int n) {
    armfw_config_t c, k;
    ff_reset();
    calstore_init(cs, &g_port, &c);
    for (int i = 1; i <= n; i++) {
        cfg_k(&k, i);
        if (run_save(cs, &k) != 1) TT_FAIL("prepare save %d", i);
    }
}

/* Tear the save of config `newk` at every budget (bytes) in every torn mode, starting from the
 * state left by `n_before` saves. After the cut and a reboot the store must hold either the
 * previous config (or defaults if there was none) or, only if the save completed, the new one;
 * and the store must keep working afterwards. */
static void torn_sweep(int n_before) {
    int outcomes[2] = {0, 0};
    for (int mode = 0; mode < FF_TORN_MODES; mode++) {
        for (int32_t budget = 0; budget <= FF_ERASE_COST + (int32_t)CALSTORE_SLOT_LEN + 8; budget++) {
            calstore_t cs;
            armfw_config_t got, prev, newk, next;
            prepare(&cs, n_before);
            if (n_before)
                cfg_k(&prev, n_before);
            else
                config_defaults(&prev);
            cfg_k(&newk, 500 + budget);
            g_ff.torn_mode = (ff_torn_mode_t)mode;
            g_ff.rng = (uint32_t)(budget * 7919 + mode);
            g_ff.budget = budget;
            int r = run_save(&cs, &newk);
            bool loaded = reboot_load(&cs, &got);
            TT_ASSERT_MSG(cfg_eq(&got, &prev) || cfg_eq(&got, &newk), "mode %d budget %d: garbage config",
                          mode, budget);
            if (r == 1) TT_ASSERT_MSG(cfg_eq(&got, &newk), "mode %d budget %d: completed save lost", mode, budget);
            if (n_before) TT_ASSERT_MSG(loaded, "mode %d budget %d: fell back to defaults", mode, budget);
            outcomes[cfg_eq(&got, &newk)]++;
            /* recovery: the next save and the one after work and survive reboots */
            for (int k2 = 0; k2 < 2; k2++) {
                cfg_k(&next, 900 + k2);
                TT_EQ(run_save(&cs, &next), 1);
                TT_ASSERT(reboot_load(&cs, &got));
                TT_ASSERT_MSG(cfg_eq(&got, &next), "mode %d budget %d: recovery save %d", mode, budget, k2);
            }
        }
    }
    TT_ASSERT(outcomes[0] > 0 && outcomes[1] > 0); /* both outcomes were exercised */
}

TT_TEST(torn_program_every_offset_first_save) { torn_sweep(0); }
TT_TEST(torn_program_every_offset_append) { torn_sweep(5); }
TT_TEST(torn_program_every_offset_last_slot) { torn_sweep(20); } /* writes the page's last slot */
TT_TEST(torn_program_every_offset_rollover) { torn_sweep(21); }  /* erase B, then write */
TT_TEST(torn_program_every_offset_second_rollover) { torn_sweep(42); } /* erase A again */

TT_TEST(torn_erase_every_double_word) {
    for (int mode = 0; mode < FF_TORN_MODES; mode++) {
        for (int32_t dw = 0; dw <= (int32_t)FF_DW_PER_PAGE; dw++) {
            calstore_t cs;
            armfw_config_t got, prev, newk;
            /* 21 + 21 saves: page A full of new records, page B full of older ones; the next save
             * must erase B, which is cut at double word dw */
            prepare(&cs, 42);
            cfg_k(&prev, 42);
            g_ff.torn_mode = (ff_torn_mode_t)mode;
            g_ff.erase_tear_dw = dw;
            g_ff.budget = 0;
            cfg_k(&newk, 1000 + dw);
            TT_EQ(run_save(&cs, &newk), 0);
            TT_ASSERT(reboot_load(&cs, &got));
            TT_ASSERT_MSG(cfg_eq(&got, &prev), "mode %d dw %d", mode, dw);
            TT_EQ(run_save(&cs, &newk), 1);
            TT_ASSERT(reboot_load(&cs, &got));
            TT_ASSERT(cfg_eq(&got, &newk));
        }
    }
}

TT_TEST(ecc_error_on_newest_record_falls_back) {
    calstore_t cs;
    armfw_config_t got, k;
    /* newest record in page B slot 0, previous one in page A slot 20 */
    prepare(&cs, 22);
    TT_EQ(cs.active_page, 1);
    g_ff.ecc_bad[1][3] = 1; /* a double word inside B slot 0 */
    TT_ASSERT(reboot_load(&cs, &got));
    cfg_k(&k, 21);
    TT_ASSERT(cfg_eq(&got, &k)); /* fell back to the other page */
    /* newest and previous both inside page A */
    prepare(&cs, 7);
    g_ff.mem[0][6 * CALSTORE_SLOT_LEN + 40] ^= 0x10; /* bit rot in slot 6 */
    TT_ASSERT(reboot_load(&cs, &got));
    cfg_k(&k, 6);
    TT_ASSERT(cfg_eq(&got, &k));
    /* the next save skips the damaged slot area (appends after it) and wins */
    cfg_k(&k, 77);
    TT_EQ(run_save(&cs, &k), 1);
    TT_ASSERT(reboot_load(&cs, &got));
    TT_ASSERT(cfg_eq(&got, &k));
}

TT_TEST(program_error_and_silent_corruption_retry) {
    calstore_t cs;
    armfw_config_t got, k;
    prepare(&cs, 3);
    uint32_t base = g_ff.programs;
    g_ff.fail_program_nth = (int32_t)base + 4; /* PROGERR on the 5th double word of this save */
    cfg_k(&k, 50);
    TT_EQ(run_save(&cs, &k), 1);
    TT_EQ(cs.burned_slots, 1);
    TT_ASSERT(reboot_load(&cs, &got));
    TT_ASSERT(cfg_eq(&got, &k));
    /* a program that reports success but stored a flipped bit: caught by the read-back */
    base = g_ff.programs;
    g_ff.corrupt_program_nth = (int32_t)base + 11;
    cfg_k(&k, 51);
    TT_EQ(run_save(&cs, &k), 1);
    TT_ASSERT(cs.burned_slots >= 1);
    TT_ASSERT(reboot_load(&cs, &got));
    TT_ASSERT(cfg_eq(&got, &k));
}

TT_TEST(persistent_failure_keeps_old_record) {
    calstore_t cs;
    armfw_config_t got, k, prev;
    prepare(&cs, 4);
    cfg_k(&prev, 4);
    /* every program fails: retries are bounded, then the save reports failure */
    for (int attempt = 0; attempt < 3; attempt++) {
        g_ff.fail_program_nth = (int32_t)g_ff.programs;
        cfg_k(&k, 60 + attempt);
        int r = 0;
        TT_EQ(calstore_save_start(&cs, &k), 0);
        while (!(r = calstore_service(&cs))) g_ff.fail_program_nth = (int32_t)g_ff.programs;
        TT_EQ(r, -1);
    }
    g_ff.fail_program_nth = -1;
    TT_ASSERT(reboot_load(&cs, &got));
    TT_ASSERT(cfg_eq(&got, &prev));
    cfg_k(&k, 70);
    TT_EQ(run_save(&cs, &k), 1);
    TT_ASSERT(reboot_load(&cs, &got));
    TT_ASSERT(cfg_eq(&got, &k));
}

TT_TEST(random_power_cut_stress) {
    /* 3000 saves with a power cut in a random place in about a third of them (several cuts can
     * hit in a row, including during recovery). Invariant after every reboot: the loaded config
     * is the last one committed or the one being written, never anything else. */
    tt_rng_t r = {2024};
    calstore_t cs;
    armfw_config_t committed, got, k;
    ff_reset();
    calstore_init(&cs, &g_port, &committed);
    int cuts = 0, completes = 0;
    for (int i = 1; i <= 3000; i++) {
        cfg_k(&k, 2000 + i);
        g_ff.torn_mode = (ff_torn_mode_t)(tt_rand(&r) % FF_TORN_MODES);
        g_ff.budget = (tt_rand(&r) % 3 == 0) ? (int32_t)(tt_rand(&r) % (CALSTORE_SLOT_LEN + 2 * FF_ERASE_COST)) : -1;
        int res = run_save(&cs, &k);
        bool died = g_ff.dead;
        reboot_load(&cs, &got);
        TT_ASSERT_MSG(cfg_eq(&got, &committed) || cfg_eq(&got, &k), "iteration %d", i);
        if (res == 1) TT_ASSERT_MSG(cfg_eq(&got, &k), "iteration %d: completed save lost", i);
        committed = got;
        cuts += died;
        completes += res == 1;
    }
    TT_ASSERT(cuts > 500 && completes > 1500);
}

TT_MAIN()
