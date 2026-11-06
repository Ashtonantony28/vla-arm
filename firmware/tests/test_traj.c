/* Trajectory: quintic boundary conditions, replan continuity, vmax stretching, limits.
 * Includes randomized property tests that drive replanning the way a policy would. */
#include <math.h>

#include "armfw/config.h"
#include "armfw/traj.h"
#include "tt.h"

static traj_limits_t default_limits(void) {
    armfw_config_t c;
    config_defaults(&c);
    traj_limits_t l;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        l.lim_min[j] = c.joint[j].limit_min_mrad;
        l.lim_max[j] = c.joint[j].limit_max_mrad;
        l.vmax[j] = c.joint[j].vmax_mrad_s;
    }
    return l;
}

TT_TEST(quintic_boundary_conditions) {
    tt_rng_t r = {1};
    for (int it = 0; it < 500; it++) {
        float p0 = (float)tt_randf(&r, -2000, 2000), v0 = (float)tt_randf(&r, -3000, 3000);
        float a0 = (float)tt_randf(&r, -30000, 30000), pf = (float)tt_randf(&r, -2000, 2000);
        float T = (float)tt_randf(&r, 0.005, 5.0);
        quintic_t q;
        quintic_plan(&q, p0, v0, a0, pf, T);
        traj_state_t s;
        quintic_eval(&q, 0.0f, &s);
        TT_NEAR(s.p, p0, 1e-3);
        TT_NEAR(s.v, v0, 1e-3 * fabs(v0) + 1e-3);
        TT_NEAR(s.a, a0, 1e-3 * fabs(a0) + 1e-2);
        /* just before T the state must approach (pf, 0, 0) */
        quintic_eval(&q, T * (1.0f - 1e-4f), &s);
        double scale = fabs(pf - p0) + fabs(v0) * T + fabs(a0) * T * T + 1;
        TT_NEAR(s.p, pf, 1e-3 * scale);
        quintic_eval(&q, T, &s);
        TT_EQ(s.p, pf);
        TT_EQ(s.v, 0);
        TT_EQ(s.a, 0);
        /* analytic end derivatives from the coefficients */
        double v1 = q.b[1] + 2 * q.b[2] + 3 * q.b[3] + 4 * q.b[4] + 5 * q.b[5];
        double a1 = 2 * q.b[2] + 6 * q.b[3] + 12 * q.b[4] + 20 * q.b[5];
        double p1 = q.b[0] + q.b[1] + q.b[2] + q.b[3] + q.b[4] + q.b[5];
        TT_NEAR(p1, pf, 1e-5 * scale);
        TT_NEAR(v1, 0, 1e-5 * scale);
        TT_NEAR(a1, 0, 1e-4 * scale);
    }
}

TT_TEST(quintic_rest_to_rest_peak_velocity) {
    quintic_t q;
    quintic_plan(&q, 0, 0, 0, 1000, 1.0f);
    TT_NEAR(quintic_peak_vel(&q), 1875.0, 0.5); /* 15/8 * D / T */
    float mn, mx;
    quintic_pos_range(&q, &mn, &mx);
    TT_NEAR(mn, 0, 1e-3);
    TT_NEAR(mx, 1000, 1e-3);
}

TT_TEST(peak_velocity_matches_dense_sampling) {
    tt_rng_t r = {3};
    for (int it = 0; it < 300; it++) {
        quintic_t q;
        quintic_plan(&q, (float)tt_randf(&r, -2000, 2000), (float)tt_randf(&r, -3000, 3000),
                     (float)tt_randf(&r, -50000, 50000), (float)tt_randf(&r, -2000, 2000),
                     (float)tt_randf(&r, 0.01, 3.0));
        double vm = 0;
        for (int i = 0; i <= 20000; i++) {
            traj_state_t s;
            quintic_eval(&q, q.T * (float)i / 20000.0f, &s);
            if (fabs(s.v) > vm) vm = fabs(s.v);
        }
        float pk = quintic_peak_vel(&q);
        TT_ASSERT_MSG(pk >= vm * (1 - 1e-4) - 1e-2, "peak %f < sampled %f", pk, vm);
        TT_ASSERT_MSG(pk <= vm * (1 + 1e-3) + 1e-2, "peak %f >> sampled %f", pk, vm);
    }
}

TT_TEST(replan_stretches_duration_for_vmax) {
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float zero[6] = {0, 0, 0, 0, 0, 800};
    traj_reset(&g, zero);
    float tgt[6] = {2000, 0, 0, 0, 0, 800};
    traj_plan_result_t r = traj_replan(&g, tgt, 0.1f, &lim);
    TT_ASSERT(r.stretched);
    TT_ASSERT(!r.pos_clamped);
    TT_ASSERT(!r.vmax_infeasible);
    double Tmin = 1.875 * 2000 / 3000.0; /* 1.25 s */
    TT_ASSERT_MSG(r.T >= Tmin * (1 - 1e-4) && r.T <= Tmin * 1.01, "T=%f", r.T);
    TT_ASSERT(r.peak_ratio <= 1.0f);
}

TT_TEST(replan_not_stretched_when_feasible) {
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float zero[6] = {0, 0, 0, 0, 0, 0};
    traj_reset(&g, zero);
    float tgt[6] = {100, -100, 50, 20, 10, 100};
    traj_plan_result_t r = traj_replan(&g, tgt, 0.5f, &lim);
    TT_ASSERT(!r.stretched);
    TT_ASSERT(!r.pos_clamped);
    TT_NEAR(r.T, 0.5, 1e-6);
    /* run to completion: ends exactly at targets */
    for (int i = 0; i < 600; i++) traj_step(&g, 1000, &lim);
    TT_ASSERT(!g.moving);
    for (int j = 0; j < 6; j++) {
        TT_EQ(g.cmd[j].p, tgt[j]);
        TT_EQ(g.cmd[j].v, 0);
        TT_EQ(g.cmd[j].a, 0);
    }
}

TT_TEST(replan_clamps_targets_to_limits) {
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float zero[6] = {0, 0, 0, 0, 0, 0};
    traj_reset(&g, zero);
    float tgt[6] = {5000, -5000, 0, 0, 0, 2000};
    traj_plan_result_t r = traj_replan(&g, tgt, 5.0f, &lim);
    TT_ASSERT(r.pos_clamped);
    for (int i = 0; i < 20000 && g.moving; i++) traj_step(&g, 1000, &lim);
    TT_EQ(g.cmd[0].p, 2600);
    TT_EQ(g.cmd[1].p, -1800);
    TT_EQ(g.cmd[5].p, 1600);
}

TT_TEST(replan_is_c2_continuous) {
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float zero[6] = {0, 0, 0, 0, 0, 0};
    traj_reset(&g, zero);
    float t1[6] = {1000, 500, -800, 300, 1000, 1200};
    traj_replan(&g, t1, 1.0f, &lim);
    for (int i = 0; i < 400; i++) traj_step(&g, 1000, &lim);
    traj_state_t before[6];
    for (int j = 0; j < 6; j++) before[j] = g.cmd[j];
    float t2[6] = {-500, -200, 400, -300, -1000, 100};
    traj_replan(&g, t2, 0.8f, &lim);
    for (int j = 0; j < 6; j++) {
        traj_state_t s;
        traj_eval_joint(&g, j, 0.0f, &s);
        TT_NEAR(s.p, before[j].p, 1e-3);
        TT_NEAR(s.v, before[j].v, 1e-2);
        TT_NEAR(s.a, before[j].a, 1e-1);
    }
}

static int g_max_evals;

/* Property test: a policy-like host replans at random rates with random targets and durations.
 * At every 1 kHz step: commanded positions within limits; |v| <= vmax (unless the planner
 * reported the move as vmax-infeasible); p, v, a continuous across replans. */
static void run_random_policy(uint64_t seed, int steps, double p_aggressive, int *n_replans,
                              int *n_infeasible, int *n_stretched, double *worst_ratio) {
    traj_limits_t lim = default_limits();
    tt_rng_t r = {seed};
    traj_group_t g;
    float start[6];
    for (int j = 0; j < 6; j++) start[j] = (float)tt_randf(&r, lim.lim_min[j], lim.lim_max[j]);
    traj_reset(&g, start);
    int next_replan = 0;
    bool infeasible_active = false;
    traj_state_t prev[6];
    for (int j = 0; j < 6; j++) prev[j] = g.cmd[j];
    for (int k = 0; k < steps; k++) {
        if (k == next_replan) {
            float tgt[6];
            for (int j = 0; j < 6; j++) {
                double span = lim.lim_max[j] - lim.lim_min[j];
                if (tt_randf(&r, 0, 1) < p_aggressive)
                    tgt[j] = (float)tt_randf(&r, lim.lim_min[j] - 0.3 * span, lim.lim_max[j] + 0.3 * span);
                else
                    tgt[j] = g.cmd[j].p + (float)tt_randf(&r, -150, 150);
            }
            float move_s = (float)(tt_randi(&r, 1, tt_randf(&r, 0, 1) < p_aggressive ? 200 : 100)) * 1e-3f;
            traj_state_t at[6];
            for (int j = 0; j < 6; j++) at[j] = g.cmd[j];
            traj_plan_result_t pr = traj_replan(&g, tgt, move_s, &lim);
            (*n_replans)++;
            if (pr.stretched) (*n_stretched)++;
            infeasible_active = pr.vmax_infeasible;
            if (pr.vmax_infeasible) (*n_infeasible)++;
            if (pr.peak_ratio > *worst_ratio) *worst_ratio = pr.peak_ratio;
            TT_ASSERT_MSG(pr.evals <= TRAJ_MAX_EVALS, "evals %d", pr.evals);
            if (pr.evals > g_max_evals) g_max_evals = pr.evals;
            for (int j = 0; j < 6; j++) {
                traj_state_t s;
                traj_eval_joint(&g, j, 0.0f, &s);
                TT_NEAR(s.p, at[j].p, 1e-2);
                TT_NEAR(s.v, at[j].v, 1e-3 * fabs(at[j].v) + 1e-2);
                TT_NEAR(s.a, at[j].a, 1e-3 * fabs(at[j].a) + 1.0);
            }
            next_replan = k + tt_randi(&r, 5, 60);
        }
        traj_step(&g, 1000, &lim);
        for (int j = 0; j < 6; j++) {
            const traj_state_t *c = &g.cmd[j];
            TT_ASSERT_MSG(c->p >= lim.lim_min[j] && c->p <= lim.lim_max[j], "j%d p=%f out of limits", j, c->p);
            {
                (void)infeasible_active;
                TT_ASSERT_MSG(fabs(c->v) <= lim.vmax[j] * 1.001 + 0.5, "step %d j%d |v|=%f > vmax %f", k, j,
                              c->v, lim.vmax[j]);
                /* finite-difference velocity agrees (no position jumps) */
                double fd = (c->p - prev[j].p) / 1e-3;
                TT_ASSERT_MSG(fabs(fd) <= lim.vmax[j] * 1.01 + 1.0, "step %d j%d fd vel %f", k, j, fd);
            }
            /* velocity continuity: |dv| over 1 ms bounded by |a| plus jerk slack */
            if (!g.saturated) {
                double dv = c->v - prev[j].v;
                /* mean value theorem: |dv| <= max|a| over the step; sample a densely */
                double amax = fmax(fabs(c->a), fabs(prev[j].a));
                for (int m = 0; m <= 20; m++) {
                    traj_state_t mid;
                    traj_eval_joint(&g, j, (float)g.elapsed_us * 1e-6f - 1e-3f * (float)m / 20.0f, &mid);
                    amax = fmax(amax, fabs(mid.a));
                }
                TT_ASSERT_MSG(fabs(dv) <= amax * 1e-3 * 1.5 + 2.0, "step %d j%d dv=%f a=%f", k, j, dv, amax);
            }
            prev[j] = *c;
        }
    }
}

TT_TEST(property_policy_like_replanning) {
    int n = 0, inf = 0, st = 0;
    double worst = 0;
    for (uint64_t seed = 1; seed <= 20; seed++) run_random_policy(seed, 5000, 0.0, &n, &inf, &st, &worst);
    printf("    policy-like: %d replans, %d stretched, %d vmax-infeasible, worst peak ratio %.5f\n", n, st, inf,
           worst);
    /* vmax may only be exceeded when replanning starts at ~vmax while still accelerating
     * outward; the excess is then bounded by a0 * 0.5 ms (see traj.h) */
    TT_ASSERT(inf * 500 < n);
    TT_ASSERT_MSG(worst <= 1.001, "worst peak ratio %f", worst);
}

TT_TEST(property_aggressive_replanning) {
    int n = 0, inf = 0, st = 0;
    double worst = 0;
    for (uint64_t seed = 100; seed < 130; seed++) run_random_policy(seed, 5000, 0.7, &n, &inf, &st, &worst);
    printf("    aggressive: %d replans, %d stretched, %d vmax-infeasible, worst peak ratio %.4f, max evals %d\n", n,
           st, inf, worst, g_max_evals);
    TT_ASSERT(st > 0);
    /* infeasible only happens when replanning at ~vmax while accelerating outward; must stay rare */
    TT_ASSERT(inf * 100 < n);
    TT_ASSERT_MSG(worst <= 1.001, "worst peak ratio %f", worst);
}

TT_TEST(freeze_stops_motion) {
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float zero[6] = {0};
    traj_reset(&g, zero);
    float t1[6] = {1000, 0, 0, 0, 0, 1000};
    traj_replan(&g, t1, 1.0f, &lim);
    for (int i = 0; i < 300; i++) traj_step(&g, 1000, &lim);
    float p = g.cmd[0].p;
    TT_ASSERT(p > 10 && p < 990);
    traj_freeze(&g);
    for (int i = 0; i < 300; i++) traj_step(&g, 1000, &lim);
    TT_EQ(g.cmd[0].p, p);
    TT_EQ(g.cmd[0].v, 0);
    TT_ASSERT(!g.moving);
}

TT_TEST(overshoot_is_saturated_at_limit) {
    /* Moving fast toward the upper limit, then a target at the limit with a long duration: the
     * quintic overshoots; the step saturates instead of leaving the limits. */
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float start[6] = {2000, 0, 0, 0, 0, 0};
    traj_reset(&g, start);
    g.cmd[0].v = 2900;
    float t1[6] = {2600, 0, 0, 0, 0, 0};
    traj_plan_result_t r = traj_replan(&g, t1, 2.0f, &lim);
    TT_ASSERT(r.overshoot);
    bool sat = false;
    while (g.moving) {
        traj_step(&g, 1000, &lim);
        sat |= g.saturated;
        TT_ASSERT(g.cmd[0].p <= 2600);
    }
    TT_ASSERT(sat);
}


TT_TEST(blend_used_when_single_quintic_infeasible) {
    /* Accelerating hard (-54 rad/s^2 at -1.9 rad/s) and then asked to go far the other way: the a0
     * term makes every single-quintic duration exceed vmax, so the planner must blend. */
    traj_limits_t lim = default_limits();
    traj_group_t g;
    float start[6] = {-1834.9f, 0, 0, 0, 0, 800};
    traj_reset(&g, start);
    g.cmd[0].v = -1937.1f;
    g.cmd[0].a = -54117.8f;
    float tgt[6] = {2600, 0, 0, 0, 0, 800};
    traj_plan_result_t r = traj_replan(&g, tgt, 0.038f, &lim);
    TT_ASSERT(r.blended);
    TT_ASSERT(!r.vmax_infeasible);
    TT_ASSERT(r.peak_ratio <= 1.0f);
    traj_state_t s0;
    traj_eval_joint(&g, 0, 0.0f, &s0);
    TT_NEAR(s0.p, -1834.9, 1e-2);
    TT_NEAR(s0.v, -1937.1, 1e-2);
    TT_NEAR(s0.a, -54117.8, 1e-1);
    /* C2 across the blend/quintic junction */
    float tau = g.blend[0].tau;
    traj_state_t a, b;
    traj_eval_joint(&g, 0, tau * (1.0f - 1e-6f), &a);
    traj_eval_joint(&g, 0, tau, &b);
    TT_NEAR(a.p, b.p, 1e-2);
    TT_NEAR(a.v, b.v, 0.5);
    TT_NEAR(a.a, b.a, 5.0);
    double vmax_seen = 0;
    while (g.moving) {
        traj_step(&g, 1000, &lim);
        if (fabs(g.cmd[0].v) > vmax_seen) vmax_seen = fabs(g.cmd[0].v);
    }
    TT_ASSERT(vmax_seen <= 3000.0 * 1.0001);
    TT_EQ(g.cmd[0].p, 2600);
}

TT_TEST(position_range_matches_dense_sampling) {
    tt_rng_t r = {11};
    for (int it = 0; it < 300; it++) {
        quintic_t q;
        quintic_plan(&q, (float)tt_randf(&r, -2000, 2000), (float)tt_randf(&r, -3000, 3000),
                     (float)tt_randf(&r, -50000, 50000), (float)tt_randf(&r, -2000, 2000),
                     (float)tt_randf(&r, 0.01, 3.0));
        double lo = 1e9, hi = -1e9;
        for (int i = 0; i <= 20000; i++) {
            traj_state_t st;
            quintic_eval(&q, q.T * (float)i / 20000.0f, &st);
            if (st.p < lo) lo = st.p;
            if (st.p > hi) hi = st.p;
        }
        float mn, mx;
        quintic_pos_range(&q, &mn, &mx);
        double tol = 1e-4 * (fabs(lo) + fabs(hi)) + 0.05;
        TT_NEAR(mn, lo, tol);
        TT_NEAR(mx, hi, tol);
    }
}

TT_MAIN()
