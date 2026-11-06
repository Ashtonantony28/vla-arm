/* Instruction-count benchmark of control_step on a Cortex-M4F model (QEMU mps2-an386).
 * QEMU runs with -icount shift=S so virtual time advances 2^S ns per instruction; SysTick (CPU
 * clock) is read around each control_step and converted to instructions with a calibration loop
 * of known length. Instructions are not cycles: on the STM32G474 (flash wait states hidden by the
 * ART accelerator, single-cycle FPU ops, 2-cycle loads) expect cycles ~ 1.2-2.0 x instructions. */
#include <stdint.h>
#include <string.h>

#include "sim.h"

#define SYST_CSR (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018u)
#define CPACR (*(volatile uint32_t *)0xE000ED88u)

static void sh_write0(const char *s) {
    register uint32_t r0 __asm("r0") = 0x04; /* SYS_WRITE0 */
    register const char *r1 __asm("r1") = s;
    __asm volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}
static void sh_exit(void) {
    register uint32_t r0 __asm("r0") = 0x18; /* SYS_EXIT */
    register uint32_t r1 __asm("r1") = 0x20026; /* ADP_Stopped_ApplicationExit */
    __asm volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}
static void put_u(uint32_t v) {
    char b[12];
    int i = 11;
    b[i] = 0;
    do {
        b[--i] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    sh_write0(&b[i]);
}

static inline uint32_t ticks(void) { return SYST_CVR; } /* counts down */
static uint32_t elapsed(uint32_t t0, uint32_t t1) { return (t0 - t1) & 0x00FFFFFFu; }

static sim_t g_sim;
static float g_ins_per_tick;

__attribute__((noinline)) static void calib_loop(uint32_t n) {
    /* 3 instructions per iteration: subs, cmp-free bne loop -> exactly 2 + overhead */
    __asm volatile("1: subs %0, %0, #1\n bne 1b" : "+r"(n));
}

static void calibrate(void) {
    uint32_t t0 = ticks();
    calib_loop(100000);
    uint32_t t1 = ticks();
    g_ins_per_tick = 200000.0f / (float)elapsed(t0, t1);
}

static uint32_t step_ins(void) {
    g_sim.now_us += CONTROL_PERIOD_US;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) fake_servo_update(&g_sim.servo[j], 1e-3f);
    uint32_t t0 = ticks();
    control_step(&g_sim.ctl);
    uint32_t t1 = ticks();
    uint8_t sink[512];
    while (sim_host_recv(&g_sim, sink, sizeof sink)) {
    }
    return (uint32_t)((float)elapsed(t0, t1) * g_ins_per_tick + 0.5f);
}

static void send_targets(const int16_t *pos, uint16_t move_ms) {
    proto_set_targets_t m = {.move_ms = move_ms, .flags = 1};
    memcpy(m.pos, pos, sizeof m.pos);
    uint8_t pl[16], fr[32];
    proto_pack_set_targets(&m, pl);
    static uint8_t seq;
    size_t n = proto_encode(fr, sizeof fr, PROTO_T_SET_TARGETS, seq++, pl, 15);
    sim_host_send(&g_sim, fr, n);
}

static uint32_t rng = 1;
static int32_t rnd(int32_t lo, int32_t hi) {
    rng = rng * 1664525u + 1013904223u;
    return lo + (int32_t)((rng >> 8) % (uint32_t)(hi - lo + 1));
}

static void report(const char *name, uint32_t sum, uint32_t n, uint32_t mx) {
    sh_write0(name);
    sh_write0(": mean ");
    put_u(sum / (n ? n : 1));
    sh_write0(" insn, max ");
    put_u(mx);
    sh_write0(" insn over ");
    put_u(n);
    sh_write0(" steps\n");
}

int main(void) {
    CPACR |= 0xFu << 20;
    __asm volatile("dsb\n isb");
    SYST_RVR = 0x00FFFFFFu;
    SYST_CVR = 0;
    SYST_CSR = 5u; /* enable, processor clock, no interrupt */
    calibrate();

    sim_init(&g_sim, 0);
    for (int i = 0; i < 20; i++) step_ins();

    /* 1. idle streaming steps */
    uint32_t sum = 0, mx = 0, n = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t c = step_ins();
        sum += c; n++; if (c > mx) mx = c;
    }
    report("idle step (torque off, STATE every 5th)", sum, n, mx);

    /* 2. 30 Hz policy: small target updates, steps between and replan steps separately */
    uint32_t rs = 0, rmx = 0, rn = 0, pev = 0;
    sum = mx = n = 0;
    int16_t pos[6] = {0, 0, 0, 0, 0, 800};
    for (int k = 0; k < 3000; k++) {
        bool rp = k % 33 == 0;
        if (rp) {
            for (int j = 0; j < 6; j++) pos[j] = (int16_t)(pos[j] + rnd(-60, 60));
            pos[5] = (int16_t)(800 + rnd(-200, 200));
            send_targets(pos, 66);
        }
        uint32_t c = step_ins();
        if (rp) { rs += c; rn++; if (c > rmx) rmx = c; pev += g_sim.ctl.last_plan.evals; }
        else { sum += c; n++; if (c > mx) mx = c; }
    }
    report("active step, no new frame", sum, n, mx);
    sh_write0("policy plan evaluations total ");
    put_u(pev);
    sh_write0("\n");
    report("active step with SET_TARGETS replan (30 Hz policy)", rs, rn, rmx);

    uint32_t ev = 0, evmx = 0;
    /* 3. adversarial: large random targets, short move_ms, replanned every 5..60 ms mid-motion */
    rs = rmx = rn = 0;
    for (int k = 0, next = 0; k < 20000; k++) {
        bool rp = k == next;
        if (rp) {
            for (int j = 0; j < 6; j++) pos[j] = (int16_t)rnd(-3000, 3000);
            pos[5] = (int16_t)rnd(-200, 1800);
            send_targets(pos, (uint16_t)rnd(1, 200));
            next = k + rnd(5, 60);
        }
        if ((k % 50) == 0 && !rp) {
            uint8_t fr[16];
            static uint8_t ps = 200;
            sim_host_send(&g_sim, fr, proto_encode(fr, sizeof fr, PROTO_T_PING, ps++, NULL, 0));
        }
        uint32_t c = step_ins();
        if (rp) { rs += c; rn++; if (c > rmx) rmx = c; ev += g_sim.ctl.last_plan.evals; if (g_sim.ctl.last_plan.evals > evmx) evmx = g_sim.ctl.last_plan.evals; }
    }
    sh_write0("adversarial plan evaluations: total ");
    put_u(ev);
    sh_write0(", max per replan ");
    put_u(evmx);
    sh_write0("\n");
    report("adversarial replans (full-range, mid-motion)", rs, rn, rmx);
    sh_write0("mode at end: ");
    put_u((uint32_t)g_sim.ctl.safety.mode);
    sh_write0(" (1 = ACTIVE)\n");
    sh_exit();
    return 0;
}
