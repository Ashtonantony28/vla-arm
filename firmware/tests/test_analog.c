/* ADC path: scaling of the rail dividers and the INA240 shunt amplifier, NTC Beta model, filtering,
 * staleness, and the safety thresholds (BUS_UNDERVOLT, 8 A / 50 ms bus overcurrent, board
 * overtemperature) driven from synthetic raw ADC code sequences. */
#include <math.h>

#include "armfw/analog.h"
#include "armfw/safety.h"
#include "tt.h"

/* Ideal code for a pin voltage (independent of the firmware's helpers). */
static uint16_t code_pin(double pin_mv, double vdda_mv) {
    double c = floor(pin_mv / vdda_mv * 4095.0 + 0.5);
    if (c < 0) c = 0;
    if (c > 4095) c = 4095;
    return (uint16_t)c;
}
static uint16_t code_bus_mv(double mv, double vdda) { return code_pin(mv * 15.0 / 115.0, vdda); }
static uint16_t code_bus_ma(double ma, double vdda) { return code_pin(ma * 0.003 * 50.0, vdda); }
/* Beta model, independent implementation with libm */
static double ntc_r(double tc) { return 10000.0 * exp(3380.0 * (1.0 / (tc + 273.15) - 1.0 / 298.15)); }
static uint16_t code_temp(double tc) {
    double r = ntc_r(tc);
    return (uint16_t)floor(4095.0 * r / (r + 10000.0) + 0.5);
}

static adc_raw_t raw_of(double bus_mv, double bus_ma, double temp_c) {
    adc_raw_t r = {0};
    r.vservo = code_bus_mv(bus_mv, 3300);
    r.vin = code_bus_mv(bus_mv, 3300);
    r.isense = code_bus_ma(bus_ma, 3300);
    r.ntc = code_temp(temp_c);
    return r;
}

typedef struct {
    analog_t a;
    safety_t s;
    armfw_config_t cfg;
    safety_inputs_t in;
} chain_t;

static void chain_init(chain_t *c) {
    analog_params_t p;
    analog_default_params(&p);
    analog_init(&c->a, &p);
    config_defaults(&c->cfg);
    memset(&c->in, 0, sizeof c->in);
    c->in.estop_loop_closed = true;
    for (int j = 0; j < 6; j++) c->in.temp_c[j] = 35;
    safety_init(&c->s, 0);
}

/* One control step: ADC sample -> analog -> safety (as control_step does). */
static void chain_step(chain_t *c, const adc_raw_t *raw) {
    analog_update(&c->a, raw);
    c->in.now_us += 1000;
    c->in.bus_mv = c->a.bus_mv;
    c->in.bus_ma = c->a.bus_ma;
    c->in.board_temp_dc = c->a.board_temp_dc;
    c->in.board_temp_valid = c->a.valid && c->a.ntc_ok;
    safety_update(&c->s, &c->cfg, &c->in);
}

TT_TEST(lnf_matches_libm) {
    double worst = 0;
    for (double x = 1e-4; x < 1e4; x *= 1.013) {
        double e = fabs(analog_lnf((float)x) - log(x));
        if (e > worst) worst = e;
    }
    TT_ASSERT_MSG(worst < 2e-6, "worst abs error %g", worst);
    TT_ASSERT(analog_lnf(0.0f) < -1e29f);
    TT_ASSERT(analog_lnf(-1.0f) < -1e29f);
}

TT_TEST(ntc_beta_model) {
    analog_params_t p;
    analog_default_params(&p);
    /* the divider midpoint is exactly R25 */
    TT_NEAR(analog_ntc_temp_c(&p, 2047.5f), 25.0, 0.01);
    /* across the useful range, against an independent libm implementation of the Beta equation
     * (inverted for exact codes, so this measures the firmware's math, not quantisation) */
    for (double tc = -30; tc <= 140; tc += 2.5) {
        double r = ntc_r(tc);
        double code = 4095.0 * r / (r + 10000.0);
        TT_NEAR(analog_ntc_temp_c(&p, (float)code), tc, 0.02);
    }
    /* with 12-bit quantisation: better than 0.25 C from 0 to 100 C */
    for (double tc = 0; tc <= 100; tc += 1) TT_NEAR(analog_ntc_temp_c(&p, code_temp(tc)), tc, 0.25);
    /* the sim's inverse helper agrees */
    for (double tc = -20; tc <= 120; tc += 7) TT_NEAR(analog_code_for_temp_c(&p, (float)tc), code_temp(tc), 1);
}

TT_TEST(scaling_bus_voltage_and_current) {
    analog_t a;
    analog_params_t p;
    analog_default_params(&p);
    for (double mv = 0; mv <= 24000; mv += 250) {
        for (double ma = 0; ma <= 20000; ma += 2500) {
            analog_init(&a, &p);
            adc_raw_t r = raw_of(mv, ma, 25);
            analog_update(&a, &r);
            /* 1 LSB = 6.2 mV on the rail, 5.4 mA on the shunt */
            TT_NEAR(a.bus_mv, mv, 7);
            TT_NEAR(a.vin_mv, mv, 7);
            TT_NEAR(a.bus_ma, ma, 6);
            TT_ASSERT(a.valid);
        }
    }
    /* the 8 A threshold sits at pin 1.2 V, code ~1489 */
    TT_NEAR(code_bus_ma(8000, 3300), 1489, 1);
    /* full scale saturates instead of wrapping */
    analog_init(&a, &p);
    adc_raw_t r = {.isense = 4095, .vservo = 4095, .vin = 4095, .ntc = 2048};
    analog_update(&a, &r);
    TT_NEAR(a.bus_mv, 3300.0 * 115.0 / 15.0, 2);
    TT_NEAR(a.bus_ma, 22000, 2);
}

TT_TEST(vrefint_corrects_vdda) {
    analog_params_t p;
    analog_default_params(&p);
    const uint16_t cal = 1655; /* typical VREFINT_CAL: 1.212 V at 3.0 V */
    for (double vdda = 3000; vdda <= 3400; vdda += 50) {
        analog_t a;
        analog_init(&a, &p);
        adc_raw_t r = {0};
        r.vservo = code_bus_mv(12000, vdda);
        r.isense = code_bus_ma(5000, vdda);
        r.ntc = code_temp(40); /* ratiometric: independent of VDDA */
        r.vrefint = (uint16_t)floor(cal * 3000.0 / vdda + 0.5);
        r.vrefint_cal = cal;
        analog_update(&a, &r);
        TT_NEAR(a.vdda_mv, vdda, 3);
        TT_NEAR(a.bus_mv, 12000, 30);
        TT_NEAR(a.bus_ma, 5000, 15);
        TT_NEAR(a.board_temp_dc, 400, 3);
        /* without VREFINT the same codes read off by the VDDA error */
        analog_t b;
        analog_init(&b, &p);
        r.vrefint = 0;
        analog_update(&b, &r);
        TT_NEAR(b.bus_mv, 12000.0 * 3300.0 / vdda, 30);
    }
}

TT_TEST(filter_and_noise) {
    analog_t a;
    analog_params_t p;
    analog_default_params(&p);
    analog_init(&a, &p);
    tt_rng_t rng = {77};
    adc_raw_t base = raw_of(12000, 2000, 30);
    int min_mv = 99999, max_mv = 0;
    for (int i = 0; i < 5000; i++) {
        adc_raw_t r = base;
        r.vservo = (uint16_t)(r.vservo + tt_randi(&rng, -8, 8)); /* +-50 mV of rail noise */
        r.isense = (uint16_t)(r.isense + tt_randi(&rng, -8, 8));
        r.ntc = (uint16_t)(r.ntc + tt_randi(&rng, -20, 20));
        analog_update(&a, &r);
        if (i > 10) {
            if (a.bus_mv < min_mv) min_mv = a.bus_mv;
            if (a.bus_mv > max_mv) max_mv = a.bus_mv;
            TT_NEAR(a.board_temp_dc, 300, 5); /* slow NTC filter: +-0.5 C despite +-20 LSB */
        }
    }
    TT_ASSERT_MSG(min_mv > 11950 && max_mv < 12050, "filtered rail %d..%d mV", min_mv, max_mv);
    /* step response: the first sample primes the filter, later steps settle within a few ms */
    analog_init(&a, &p);
    adc_raw_t hi = raw_of(12000, 0, 25), lo = raw_of(10000, 0, 25);
    analog_update(&a, &hi);
    TT_NEAR(a.bus_mv, 12000, 7);
    int n = 0;
    while (a.bus_mv > 10050 && n < 100) {
        analog_update(&a, &lo);
        n++;
    }
    TT_ASSERT_MSG(n <= 7, "settled to 50 mV in %d ms", n);
}

TT_TEST(stale_adc_reads_zero) {
    analog_t a;
    analog_params_t p;
    analog_default_params(&p);
    analog_init(&a, &p);
    adc_raw_t r = raw_of(12000, 3000, 30);
    analog_update(&a, &r);
    for (unsigned i = 0; i < ANALOG_STALE_MS - 1; i++) analog_update(&a, NULL);
    TT_ASSERT(a.valid); /* short gaps keep the last reading */
    TT_NEAR(a.bus_mv, 12000, 7);
    analog_update(&a, NULL);
    TT_ASSERT(!a.valid);
    TT_EQ(a.bus_mv, 0);
    TT_EQ(a.bus_ma, 0);
    analog_update(&a, &r); /* recovers on the next conversion */
    TT_ASSERT(a.valid);
    TT_NEAR(a.bus_mv, 12000, 7);
}

TT_TEST(undervolt_from_adc) {
    chain_t c;
    chain_init(&c);
    adc_raw_t ok = raw_of(12000, 500, 30), low = raw_of(10200, 500, 30), band = raw_of(10600, 500, 30);
    for (int i = 0; i < 100; i++) chain_step(&c, &ok);
    TT_EQ(c.s.faults, 0);
    /* a 3 ms dip below 10.5 V is ignored */
    for (int i = 0; i < 3; i++) chain_step(&c, &low);
    for (int i = 0; i < 10; i++) chain_step(&c, &ok);
    TT_EQ(c.s.faults, 0);
    /* sustained: set after filter settling (2 ms) + 5 ms */
    int n = 0;
    while (!(c.s.faults & FAULT_BUS_UNDERVOLT) && n < 100) {
        chain_step(&c, &low);
        n++;
    }
    TT_ASSERT_MSG(n >= SAFETY_UNDERVOLT_SET_MS && n <= SAFETY_UNDERVOLT_SET_MS + 4, "set after %d ms", n);
    TT_EQ(c.s.mode, MODE_IDLE); /* informational only */
    /* inside the hysteresis band it stays set */
    for (int i = 0; i < 200; i++) chain_step(&c, &band);
    TT_ASSERT(c.s.faults & FAULT_BUS_UNDERVOLT);
    n = 0;
    while ((c.s.faults & FAULT_BUS_UNDERVOLT) && n < 200) {
        chain_step(&c, &ok);
        n++;
    }
    TT_ASSERT_MSG(n >= SAFETY_UNDERVOLT_CLEAR_MS && n <= SAFETY_UNDERVOLT_CLEAR_MS + 4, "cleared after %d", n);
    /* a dead ADC reads 0 V and raises the flag (visible sensing failure) but no overcurrent */
    for (int i = 0; i < 30; i++) chain_step(&c, NULL);
    TT_EQ(c.s.faults, FAULT_BUS_UNDERVOLT);
}

TT_TEST(bus_overcurrent_8a_50ms_from_adc) {
    chain_t c;
    chain_init(&c);
    adc_raw_t ok = raw_of(12000, 3000, 30), over = raw_of(11800, 8500, 30), under = raw_of(11800, 7800, 30);
    adc_raw_t spike = raw_of(11500, 20000, 30);
    for (int i = 0; i < 50; i++) chain_step(&c, &ok);
    /* 7.8 A forever: no trip */
    for (int i = 0; i < 2000; i++) chain_step(&c, &under);
    TT_EQ(c.s.faults, 0);
    /* single-sample 20 A spikes every 10 ms (inrush, noise): no trip */
    for (int k = 0; k < 100; k++) {
        chain_step(&c, &spike);
        for (int i = 0; i < 9; i++) chain_step(&c, &ok);
    }
    TT_EQ(c.s.faults, 0);
    /* 40 ms bursts of 8.5 A separated by 10 ms at 3 A: no trip */
    for (int k = 0; k < 20; k++) {
        for (int i = 0; i < 40; i++) chain_step(&c, &over);
        for (int i = 0; i < 10; i++) chain_step(&c, &ok);
    }
    TT_EQ(c.s.faults, 0);
    /* sustained 8.5 A from 3 A: latches after 50 ms + the IIR crossing delay (3 ms for this step) */
    safety_request_torque(&c.s, true, c.in.now_us);
    int n = 0;
    while (!(c.s.faults & FAULT_OVERCURRENT) && n < 200) {
        c.s.last_host_us = c.in.now_us; /* keep the watchdog fed */
        chain_step(&c, &over);
        n++;
    }
    TT_ASSERT_MSG(n >= SAFETY_BUS_OC_MS && n <= SAFETY_BUS_OC_MS + 4, "tripped after %d ms", n);
    TT_EQ(c.s.mode, MODE_FAULT);
    /* clear refused while still over, accepted once the current is back */
    safety_clear(&c.s, FAULT_OVERCURRENT);
    TT_ASSERT(c.s.faults & FAULT_OVERCURRENT);
    for (int i = 0; i < 5; i++) chain_step(&c, &ok);
    safety_clear(&c.s, FAULT_OVERCURRENT);
    TT_EQ(c.s.faults & FAULT_OVERCURRENT, 0);
}

TT_TEST(board_overtemp_from_ntc) {
    chain_t c;
    chain_init(&c);
    adc_raw_t warm = raw_of(12000, 1000, 70), hot = raw_of(12000, 1000, 95), mid = raw_of(12000, 1000, 82);
    for (int i = 0; i < 3000; i++) chain_step(&c, &warm);
    TT_EQ(c.s.faults, 0);
    TT_NEAR(c.a.board_temp_dc, 700, 3);
    int n = 0;
    while (!(c.s.faults & FAULT_OVERTEMP) && n < 5000) {
        chain_step(&c, &hot);
        n++;
    }
    /* NTC filter (tau 64 ms) to cross 85 C, then 1 s */
    TT_ASSERT_MSG(n > SAFETY_BOARD_OT_MS && n < SAFETY_BOARD_OT_MS + 300, "tripped after %d ms", n);
    TT_EQ(c.s.mode, MODE_FAULT);
    /* clearing is refused while the board is still >= 80 C */
    for (int i = 0; i < 2000; i++) chain_step(&c, &mid);
    safety_clear(&c.s, FAULT_OVERTEMP);
    TT_ASSERT(c.s.faults & FAULT_OVERTEMP);
    for (int i = 0; i < 2000; i++) chain_step(&c, &warm);
    safety_clear(&c.s, FAULT_OVERTEMP);
    TT_EQ(c.s.faults, 0);
}

TT_TEST(broken_ntc_never_trips) {
    chain_t c;
    chain_init(&c);
    adc_raw_t shorted = raw_of(12000, 0, 30), open = raw_of(12000, 0, 30);
    shorted.ntc = 0; /* reads as absurdly hot */
    open.ntc = 4095;
    for (int i = 0; i < 3000; i++) chain_step(&c, &shorted);
    TT_ASSERT(!c.a.ntc_ok);
    TT_EQ(c.s.faults, 0);
    for (int i = 0; i < 3000; i++) chain_step(&c, &open);
    TT_ASSERT(!c.a.ntc_ok);
    TT_EQ(c.s.faults, 0);
    /* a near-short (a few LSB) is still outside the plausible range */
    shorted.ntc = 3;
    for (int i = 0; i < 3000; i++) chain_step(&c, &shorted);
    TT_ASSERT(!c.a.ntc_ok);
    TT_EQ(c.s.faults, 0);
}

TT_MAIN()
