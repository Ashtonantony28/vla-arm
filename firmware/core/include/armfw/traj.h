/* Per-joint quintic trajectories with replanning (spec/protocol.md, SET_TARGETS).
 *
 * A move from the current commanded state (p0, v0, a0) to (pf, 0, 0) in duration T is a quintic in
 * normalised time s = t/T:  P(s) = p0 + V0 s + A0/2 s^2 + b3 s^3 + b4 s^4 + b5 s^5,
 * with V0 = v0 T, A0 = a0 T^2. Replanning mid-move starts from the commanded p, v, a, so the
 * command is continuous to 2nd order.
 *
 * All joints of one SET_TARGETS share one end time. Targets are clamped to the joint limits.
 * If any joint's peak |velocity| would exceed its vmax, the shared duration is stretched until no
 * joint exceeds vmax (trajectory shape is kept, nothing is clipped).
 *
 * Feasibility gap (reported, see README): from a start state with large acceleration a0, the a0
 * term of the quintic grows linearly with T, so for some replans no single-quintic duration
 * respects vmax. Only in that case the planner prepends a short per-joint "blend" (constant jerk,
 * acceleration ramps linearly from a0 to 0, still C2-continuous) and then plans the stretched
 * quintic from the blend's end state, ending at the same common time for all joints.
 *
 * Units: positions in mrad, velocities mrad/s, accelerations mrad/s^2, time in seconds (float).
 */
#ifndef ARMFW_TRAJ_H
#define ARMFW_TRAJ_H

#include "armfw/common.h"

#define TRAJ_MAX_DURATION_S 60.0f /* hard cap on stretched duration */
#define TRAJ_BLEND_MAX_S 0.020f   /* longest acceleration-removal blend */
#define TRAJ_BLEND_VFRAC 0.95f    /* blend keeps |v| below this fraction of vmax when possible */
#define TRAJ_SEARCH_ITERS 8        /* duration refinements per stage (single quintic, then blend) */
#define TRAJ_MAX_EVALS (2 * (TRAJ_SEARCH_ITERS + 2) + 1) /* worst-case plan evaluations per replan */

typedef struct {
    float p, v, a;
} traj_state_t;

typedef struct {
    float b[6]; /* normalised coefficients, P(s) = sum b[k] s^k */
    float T;    /* seconds, > 0 */
    float pf;   /* exact end position (returned for t >= T) */
} quintic_t;

/* Constant-jerk segment for t in [0, tau): a(t) = a0 + jerk t. tau == 0 means no blend. */
typedef struct {
    float tau;
    float p0, v0, a0, jerk;
} traj_blend_t;

/* Plan a single quintic (p0,v0,a0) -> (pf,0,0) in T seconds. */
void quintic_plan(quintic_t *q, float p0, float v0, float a0, float pf, float T);
/* Evaluate at time t (seconds). t >= T yields (pf, 0, 0); t <= 0 yields the start state. */
void quintic_eval(const quintic_t *q, float t, traj_state_t *out);
/* Peak |velocity| over [0, T] (mrad/s). */
float quintic_peak_vel(const quintic_t *q);
/* Position range over [0, T]. */
void quintic_pos_range(const quintic_t *q, float *pmin, float *pmax);

typedef struct {
    bool pos_clamped;     /* at least one target was outside its limits and was clamped */
    bool stretched;       /* duration was stretched to respect vmax */
    bool blended;         /* single quintic infeasible: acceleration-removal blend used */
    bool vmax_infeasible; /* even the blended plan exceeds vmax (e.g. |v0| > vmax); best effort */
    bool overshoot;       /* the planned path leaves the limits (start velocity); saturated */
    float T;              /* total duration actually used, seconds */
    float peak_ratio;     /* max over joints of peak|v| / vmax for the plan used */
    uint8_t evals;        /* candidate-duration evaluations spent (bounded, see TRAJ_MAX_EVALS) */
} traj_plan_result_t;

typedef struct {
    quintic_t q[ARMFW_N_JOINTS];
    traj_blend_t blend[ARMFW_N_JOINTS];
    float t0[ARMFW_N_JOINTS];         /* start time of q[j] within the move, seconds */
    traj_state_t cmd[ARMFW_N_JOINTS]; /* current commanded state */
    uint32_t elapsed_us;
    bool moving;
    bool saturated; /* last step clamped a commanded position at a limit */
} traj_group_t;

typedef struct {
    float lim_min[ARMFW_N_JOINTS];
    float lim_max[ARMFW_N_JOINTS];
    float vmax[ARMFW_N_JOINTS];
} traj_limits_t;

/* Hold all joints at pos with zero velocity/acceleration (no motion). */
void traj_reset(traj_group_t *g, const float pos[ARMFW_N_JOINTS]);
/* Stop where the command currently is: freeze position, zero velocity and acceleration. */
void traj_freeze(traj_group_t *g);
/* Replan from the current commanded state to target (mrad) over move_s seconds. */
traj_plan_result_t traj_replan(traj_group_t *g, const float target[ARMFW_N_JOINTS], float move_s,
                               const traj_limits_t *lim);
/* Evaluate joint j of the current plan at move time t (seconds). */
void traj_eval_joint(const traj_group_t *g, int j, float t, traj_state_t *out);
/* Advance time by dt_us and update cmd[]. Commanded positions are saturated to the limits
 * (sets g->saturated). This only happens when a replan starts with velocity toward a nearby limit
 * and no quintic can stop in time (traj_plan_result_t.overshoot). The saturated joint stops at
 * the limit and is replanned from rest there to its target (rest-to-rest, within vmax), so it
 * does not re-emerge from the limit with a velocity jump. */
void traj_step(traj_group_t *g, uint32_t dt_us, const traj_limits_t *lim);

#endif
