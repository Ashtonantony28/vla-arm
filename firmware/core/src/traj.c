#include "armfw/traj.h"

#define ROOT_ITERS 8 /* safeguarded regula falsi (Illinois) iterations per bracketed root */

static float poly_eval(const float *c, int deg, float s) {
    float r = c[deg];
    for (int k = deg - 1; k >= 0; k--) r = r * s + c[k];
    return r;
}

static void poly_deriv(const float *c, int deg, float *dc) {
    for (int k = 1; k <= deg; k++) dc[k - 1] = (float)k * c[k];
}

/* Sorted points in the open interval (0, 1). */
typedef struct {
    float s[4];
    int n;
} pts01_t;

static void pts_add(pts01_t *p, float s) {
    if (!(s > 0.0f && s < 1.0f) || p->n >= 4) return;
    int i = p->n++;
    while (i > 0 && p->s[i - 1] > s) {
        p->s[i] = p->s[i - 1];
        i--;
    }
    p->s[i] = s;
}

/* Roots in (0,1) of c0 + c1 s + c2 s^2, cancellation-free form. The square root is a single VSQRT
 * on the M4F (the core is built with -fno-math-errno, so no libm call). */
static void quad_roots01(const float *c, pts01_t *out) {
    out->n = 0;
    float a = c[2], b = c[1], k = c[0];
    float scale = armfw_fmaxf(armfw_fmaxf(armfw_fabsf(a), armfw_fabsf(b)), armfw_fabsf(k));
    if (scale == 0.0f) return;
    if (armfw_fabsf(a) <= 1e-7f * scale) {
        if (b != 0.0f) pts_add(out, -k / b);
        return;
    }
    float disc = b * b - 4.0f * a * k;
    if (disc < 0.0f) return;
    float sq = __builtin_sqrtf(disc);
    float q = -0.5f * (b + (b >= 0.0f ? sq : -sq));
    if (q != 0.0f) {
        pts_add(out, q / a);
        pts_add(out, k / q);
    }
}

/* Roots in (0,1) of polynomial c (degree deg), given the sorted critical points of c in (0,1):
 * c is monotone between consecutive knots, so each interval holds at most one root, found by
 * the Illinois variant of regula falsi with a bisection safeguard. */
static void monotone_roots01(const float *c, int deg, const pts01_t *crit, pts01_t *out) {
    out->n = 0;
    float a = 0.0f, fa = poly_eval(c, deg, 0.0f);
    for (int i = 0; i <= crit->n; i++) {
        float b = i < crit->n ? crit->s[i] : 1.0f;
        float fb = poly_eval(c, deg, b);
        if (fb == 0.0f) {
            pts_add(out, b);
        } else if ((fa < 0.0f && fb > 0.0f) || (fa > 0.0f && fb < 0.0f)) {
            float lo = a, hi = b, flo = fa, fhi = fb, x = 0.5f * (a + b);
            int side = 0;
            for (int it = 0; it < ROOT_ITERS; it++) {
                x = (lo * fhi - hi * flo) / (fhi - flo);
                if (!(x > lo && x < hi)) x = 0.5f * (lo + hi);
                float fx = poly_eval(c, deg, x);
                if (fx == 0.0f) break;
                if ((fx < 0.0f) == (flo < 0.0f)) {
                    lo = x;
                    flo = fx;
                    if (side == -1) fhi *= 0.5f;
                    side = -1;
                } else {
                    hi = x;
                    fhi = fx;
                    if (side == 1) flo *= 0.5f;
                    side = 1;
                }
                if (hi - lo < 1e-4f) break; /* extremum value error ~ (1e-4)^2 */
            }
            pts_add(out, x);
        }
        a = b;
        fa = fb;
    }
}

/* Extrema of a quintic in normalised time. Velocity extrema sit at roots of the cubic P2 = P'',
 * whose critical points are the roots of the quadratic P3 = P''' (closed form). Position extrema
 * sit at roots of the quartic P1 = P', which is monotone between roots of P2. No sampling. */
static void quintic_extrema(const quintic_t *q, float *vpeak, float *pmin, float *pmax) {
    float d1[5], d2[4], d3[3];
    poly_deriv(q->b, 5, d1);
    poly_deriv(d1, 4, d2);
    poly_deriv(d2, 3, d3);
    pts01_t r3, r2;
    quad_roots01(d3, &r3);
    monotone_roots01(d2, 3, &r3, &r2);
    if (vpeak) {
        float m = armfw_fmaxf(armfw_fabsf(d1[0]), armfw_fabsf(poly_eval(d1, 4, 1.0f)));
        for (int i = 0; i < r2.n; i++) m = armfw_fmaxf(m, armfw_fabsf(poly_eval(d1, 4, r2.s[i])));
        *vpeak = m / q->T;
    }
    if (pmin && pmax) {
        pts01_t r1;
        monotone_roots01(d1, 4, &r2, &r1);
        float lo = armfw_fminf(q->b[0], q->pf), hi = armfw_fmaxf(q->b[0], q->pf);
        for (int i = 0; i < r1.n; i++) {
            float v = poly_eval(q->b, 5, r1.s[i]);
            lo = armfw_fminf(lo, v);
            hi = armfw_fmaxf(hi, v);
        }
        *pmin = lo;
        *pmax = hi;
    }
}

void quintic_plan(quintic_t *q, float p0, float v0, float a0, float pf, float T) {
    if (!(T > 1e-6f)) T = 1e-6f;
    float V0 = v0 * T;
    float A0 = a0 * T * T;
    float h = pf - p0 - V0 - 0.5f * A0; /* b3 + b4 + b5 */
    float g = -V0 - A0;                 /* 3b3 + 4b4 + 5b5 */
    float k = -A0;                      /* 6b3 + 12b4 + 20b5 */
    q->b[0] = p0;
    q->b[1] = V0;
    q->b[2] = 0.5f * A0;
    q->b[3] = 10.0f * h - 4.0f * g + 0.5f * k;
    q->b[4] = -15.0f * h + 7.0f * g - k;
    q->b[5] = 6.0f * h - 3.0f * g + 0.5f * k;
    q->T = T;
    q->pf = pf;
}

void quintic_eval(const quintic_t *q, float t, traj_state_t *out) {
    if (t >= q->T) {
        out->p = q->pf;
        out->v = 0.0f;
        out->a = 0.0f;
        return;
    }
    if (t < 0.0f) t = 0.0f;
    float s = t / q->T;
    float d1[5], d2[4];
    poly_deriv(q->b, 5, d1);
    poly_deriv(d1, 4, d2);
    float invT = 1.0f / q->T;
    out->p = poly_eval(q->b, 5, s);
    out->v = poly_eval(d1, 4, s) * invT;
    out->a = poly_eval(d2, 3, s) * invT * invT;
}

float quintic_peak_vel(const quintic_t *q) {
    float v;
    quintic_extrema(q, &v, NULL, NULL);
    return v;
}

void quintic_pos_range(const quintic_t *q, float *pmin, float *pmax) { quintic_extrema(q, NULL, pmin, pmax); }

static void blend_eval(const traj_blend_t *b, float t, traj_state_t *out) {
    out->p = b->p0 + t * (b->v0 + t * (0.5f * b->a0 + t * (b->jerk * (1.0f / 6.0f))));
    out->v = b->v0 + t * (b->a0 + 0.5f * b->jerk * t);
    out->a = b->a0 + b->jerk * t;
}

void traj_eval_joint(const traj_group_t *g, int j, float t, traj_state_t *out) {
    if (t < g->blend[j].tau) {
        blend_eval(&g->blend[j], t < 0.0f ? 0.0f : t, out);
    } else {
        quintic_eval(&g->q[j], t - g->t0[j], out);
    }
}

void traj_reset(traj_group_t *g, const float pos[ARMFW_N_JOINTS]) {
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        g->cmd[j].p = pos[j];
        g->cmd[j].v = 0.0f;
        g->cmd[j].a = 0.0f;
        quintic_plan(&g->q[j], pos[j], 0.0f, 0.0f, pos[j], 1e-3f);
        g->t0[j] = 0.0f;
        g->blend[j].tau = 0.0f;
    }
    g->elapsed_us = 0;
    g->moving = false;
    g->saturated = false;
}

void traj_freeze(traj_group_t *g) {
    float pos[ARMFW_N_JOINTS];
    for (int j = 0; j < ARMFW_N_JOINTS; j++) pos[j] = g->cmd[j].p;
    traj_reset(g, pos);
}

/* Plan every joint so that all end at T_total. With blends, joint j first runs its blend of length
 * blend[j].tau, then a quintic of T_total - tau. Returns max over joints of peak|v| / vmax. */
static float plan_all(traj_group_t *g, const float *tgt, float T_total, bool use_blend,
                      const traj_limits_t *lim) {
    float ratio = 0.0f;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        const traj_state_t *c = &g->cmd[j];
        float vmax = armfw_fmaxf(lim->vmax[j], 1.0f);
        float peak;
        if (use_blend && g->blend[j].tau > 0.0f) {
            traj_blend_t *b = &g->blend[j];
            traj_state_t e;
            blend_eval(b, b->tau, &e);
            quintic_plan(&g->q[j], e.p, e.v, 0.0f, tgt[j], T_total - b->tau);
            g->t0[j] = b->tau;
            /* acceleration keeps its sign during the blend, so |v| is monotone there */
            peak = armfw_fmaxf(armfw_fmaxf(armfw_fabsf(b->v0), armfw_fabsf(e.v)), quintic_peak_vel(&g->q[j]));
        } else {
            quintic_plan(&g->q[j], c->p, c->v, c->a, tgt[j], T_total);
            g->t0[j] = 0.0f;
            peak = quintic_peak_vel(&g->q[j]);
        }
        ratio = armfw_fmaxf(ratio, peak / vmax);
    }
    return ratio;
}

/* Find a short T >= T_min with ratio <= 1. With x = 1/(T - fixed), the distance term of the peak
 * velocity is proportional to x, while the start-velocity term does not depend on T, so
 * ratio(x) is close to linear: one multiplicative step, then secant steps on x. A start
 * acceleration adds a term growing with T, so the search stops as soon as stretching no longer
 * helps. Work is bounded by TRAJ_SEARCH_ITERS + 2 evaluations. Leaves g planned with the
 * returned T. */
static float search_T(traj_group_t *g, const float *tgt, float T_min, float fixed, bool use_blend,
                      const traj_limits_t *lim, float *ratio_out, uint8_t *evals) {
    const float goal = 0.999f;
    float T = T_min;
    float ratio = plan_all(g, tgt, T, use_blend, lim);
    (*evals)++;
    float best_T = T, best_ratio = ratio;
    float T_prev = 0.0f, r_prev = 0.0f;
    for (int it = 0; it < TRAJ_SEARCH_ITERS && ratio > 1.0f && T < TRAJ_MAX_DURATION_S; it++) {
        float span = T - fixed;
        float Tn;
        if (it == 0 || !(r_prev > ratio)) {
            Tn = fixed + span * armfw_fminf(ratio / goal, 8.0f);
        } else {
            float x1 = 1.0f / (T_prev - fixed), x2 = 1.0f / span;
            float xn = x2 + (goal - ratio) * (x2 - x1) / (ratio - r_prev);
            Tn = xn > 0.0f ? fixed + 1.0f / xn : fixed + span * 8.0f;
            Tn = armfw_clampf(Tn, fixed + span * 1.001f, fixed + span * 8.0f);
        }
        T_prev = T;
        r_prev = ratio;
        T = armfw_fminf(Tn, TRAJ_MAX_DURATION_S);
        ratio = plan_all(g, tgt, T, use_blend, lim);
        (*evals)++;
        if (ratio < best_ratio) {
            best_ratio = ratio;
            best_T = T;
        }
        if (ratio > 1.0f && ratio >= r_prev) break; /* stretching no longer helps */
    }
    if (best_T != T) {
        T = best_T;
        ratio = plan_all(g, tgt, T, use_blend, lim);
        (*evals)++;
    }
    *ratio_out = ratio;
    return T;
}

/* Choose each joint's blend: ramp acceleration a0 -> 0 over tau, keeping |v| <= 0.95 vmax where
 * the start state allows. */
static float choose_blends(traj_group_t *g, const traj_limits_t *lim) {
    float max_tau = 0.0f;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        const traj_state_t *c = &g->cmd[j];
        traj_blend_t *b = &g->blend[j];
        b->p0 = c->p;
        b->v0 = c->v;
        b->a0 = c->a;
        float tau = 0.0f;
        if (armfw_fabsf(c->a) > 1.0f) {
            tau = TRAJ_BLEND_MAX_S;
            float vcap = TRAJ_BLEND_VFRAC * armfw_fmaxf(lim->vmax[j], 1.0f);
            bool outward = (c->a > 0.0f) == (c->v > 0.0f);
            if (outward) {
                /* v(tau) = v0 + a0 tau / 2 must stay within vcap */
                float room = vcap - armfw_fabsf(c->v);
                float tau_v = room > 0.0f ? 2.0f * room / armfw_fabsf(c->a) : 0.0f;
                tau = armfw_fminf(tau, tau_v);
            }
            tau = armfw_fmaxf(tau, 1e-3f);
            b->jerk = -c->a / tau;
        } else {
            b->jerk = 0.0f;
        }
        b->tau = tau;
        max_tau = armfw_fmaxf(max_tau, tau);
    }
    return max_tau;
}

traj_plan_result_t traj_replan(traj_group_t *g, const float target[ARMFW_N_JOINTS], float move_s,
                               const traj_limits_t *lim) {
    traj_plan_result_t r = {false, false, false, false, false, 0.0f, 0.0f, 0};
    float tgt[ARMFW_N_JOINTS];
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        tgt[j] = armfw_clampf(target[j], lim->lim_min[j], lim->lim_max[j]);
        if (tgt[j] != target[j]) r.pos_clamped = true;
        g->blend[j].tau = 0.0f;
    }
    float T0 = armfw_clampf(move_s, 1e-3f, TRAJ_MAX_DURATION_S);
    float ratio;
    float T = search_T(g, tgt, T0, 0.0f, false, lim, &ratio, &r.evals);
    if (ratio > 1.0f) {
        float max_tau = choose_blends(g, lim);
        if (max_tau > 0.0f) {
            float ratio_b;
            float Tb = search_T(g, tgt, armfw_fmaxf(T0, max_tau + 1e-3f), max_tau, true, lim, &ratio_b, &r.evals);
            if (ratio_b < ratio) {
                r.blended = true;
                T = Tb;
                ratio = ratio_b;
            } else {
                for (int j = 0; j < ARMFW_N_JOINTS; j++) g->blend[j].tau = 0.0f;
                ratio = plan_all(g, tgt, T, false, lim);
                r.evals++;
            }
        }
        r.vmax_infeasible = ratio > 1.0f;
    }
    r.T = T;
    r.peak_ratio = ratio;
    r.stretched = T > T0 * 1.000001f;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        float pmin, pmax;
        quintic_pos_range(&g->q[j], &pmin, &pmax);
        if (pmin < lim->lim_min[j] - 0.01f || pmax > lim->lim_max[j] + 0.01f) r.overshoot = true;
    }
    g->elapsed_us = 0;
    g->moving = true;
    return r;
}

void traj_step(traj_group_t *g, uint32_t dt_us, const traj_limits_t *lim) {
    g->saturated = false;
    float t = 0.0f;
    if (g->moving) {
        g->elapsed_us += dt_us;
        t = (float)g->elapsed_us * 1e-6f;
        bool done = true;
        for (int j = 0; j < ARMFW_N_JOINTS; j++) {
            traj_eval_joint(g, j, t, &g->cmd[j]);
            if (t < g->t0[j] + g->q[j].T) done = false;
        }
        if (done) g->moving = false;
    }
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        traj_state_t *c = &g->cmd[j];
        if (c->p < lim->lim_min[j] || c->p > lim->lim_max[j]) {
            c->p = armfw_clampf(c->p, lim->lim_min[j], lim->lim_max[j]);
            c->v = 0.0f;
            c->a = 0.0f;
            g->saturated = true;
            if (g->moving) {
                /* Continue to the (in-limit) target from rest at the limit. */
                quintic_t *q = &g->q[j];
                float pf = q->pf;
                float rem = g->t0[j] + q->T - t;
                float vmax = armfw_fmaxf(lim->vmax[j], 1.0f);
                float tmin = 1.875f * armfw_fabsf(pf - c->p) / vmax * 1.002f;
                quintic_plan(q, c->p, 0.0f, 0.0f, pf, armfw_fmaxf(armfw_fmaxf(rem, tmin), 1e-3f));
                g->t0[j] = t;
                g->blend[j].tau = 0.0f;
            }
        }
    }
}
