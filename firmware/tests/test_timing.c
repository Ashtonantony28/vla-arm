/* Host-side cost of one control step (the full 1 kHz step: servo RX parse, host frame handling,
 * safety, trajectory, servo TX encode, STATE encode). This is a host CPU number, not a Cortex-M4
 * number; it is a regression guard and a rough budget check. Also times the replan path. */
#include <time.h>

#include "host_helpers.h"

#if defined(__SANITIZE_ADDRESS__)
#define TIMING_SLACK 20.0 /* ASan/UBSan builds are much slower */
#else
#define TIMING_SLACK 1.0
#endif

static sim_t g_sim;
static host_t g_host;

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static int cmp_d(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

#define N_STEPS 20000
static double g_ns[N_STEPS];
static double g_replan_ns[N_STEPS];

TT_TEST(control_step_cost) {
    sim_init(&g_sim, 0);
    hh_init(&g_host, &g_sim);
    hh_run(&g_host, 20);
    int n_replan = 0;
    for (int k = 0; k < N_STEPS; k++) {
        bool replan = k % 33 == 0;
        if (replan) {
            int16_t t[6];
            for (int j = 0; j < 6; j++) t[j] = (int16_t)((k * 7 + j * 131) % 1200 - 600);
            t[5] = (int16_t)(800 + (k % 400));
            hh_set_targets(&g_host, 66, t, PROTO_FLAG_TORQUE);
        }
        /* sim_step without the timing of the fake servo physics */
        g_sim.now_us += CONTROL_PERIOD_US;
        for (int j = 0; j < 6; j++) fake_servo_update(&g_sim.servo[j], 1e-3f);
        double t0 = now_ns();
        control_step(&g_sim.ctl);
        double dt = now_ns() - t0;
        g_ns[k] = dt;
        if (replan) g_replan_ns[n_replan++] = dt;
        hh_drain(&g_host);
    }
    TT_EQ(g_sim.ctl.safety.mode, MODE_ACTIVE);
    double sum = 0;
    for (int k = 0; k < N_STEPS; k++) sum += g_ns[k];
    qsort(g_ns, N_STEPS, sizeof(double), cmp_d);
    qsort(g_replan_ns, (size_t)n_replan, sizeof(double), cmp_d);
    double mean = sum / N_STEPS / 1000.0;
    double p50 = g_ns[N_STEPS / 2] / 1000.0, p99 = g_ns[N_STEPS * 99 / 100] / 1000.0;
    double mx = g_ns[N_STEPS - 1] / 1000.0;
    double rp50 = g_replan_ns[n_replan / 2] / 1000.0, rmax = g_replan_ns[n_replan - 1] / 1000.0;
    printf("    control_step on host: mean %.2f us, p50 %.2f us, p99 %.2f us, max %.2f us\n", mean, p50, p99, mx);
    printf("    steps with a SET_TARGETS replan (%d): p50 %.2f us, max %.2f us\n", n_replan, rp50, rmax);
    TT_ASSERT(mean < 50.0 * TIMING_SLACK);
    TT_ASSERT(p99 < 200.0 * TIMING_SLACK);
}

TT_MAIN()
