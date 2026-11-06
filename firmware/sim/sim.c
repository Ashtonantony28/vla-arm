#include "sim.h"

#include <string.h>

/* ---------------- fifo ---------------- */

static void fifo_init(sim_fifo_t *f, uint32_t cap) {
    f->head = f->tail = 0;
    f->cap = cap;
}
static size_t fifo_write(sim_fifo_t *f, const uint8_t *d, size_t n) {
    size_t w = 0;
    while (w < n && f->tail - f->head < f->cap) {
        f->buf[f->tail % f->cap] = d[w++];
        f->tail++;
    }
    return w;
}
static size_t fifo_read(sim_fifo_t *f, uint8_t *d, size_t cap) {
    size_t r = 0;
    while (r < cap && f->head != f->tail) {
        d[r++] = f->buf[f->head % f->cap];
        f->head++;
    }
    return r;
}

/* ---------------- fake HAL ---------------- */

static uint32_t h_time_us(void *ctx) { return ((sim_t *)ctx)->now_us; }

static size_t h_host_read(void *ctx, uint8_t *buf, size_t cap) {
    return fifo_read(&((sim_t *)ctx)->host_in, buf, cap);
}

static size_t h_host_write(void *ctx, const uint8_t *buf, size_t n) {
    return fifo_write(&((sim_t *)ctx)->host_out, buf, n);
}

static void rx_append(sim_t *s, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n && s->servo_rx_len < SIM_SERVO_RX_CAP; i++) s->servo_rx[s->servo_rx_len++] = d[i];
}

static void h_servo_txrx(void *ctx, const uint8_t *tx, size_t n) {
    sim_t *s = (sim_t *)ctx;
    s->last_tx_bytes = (uint32_t)n;
    if (n > s->max_tx_bytes) s->max_tx_bytes = (uint32_t)n;
    if (s->echo_tx) rx_append(s, tx, n);
    sb_packet_t pk;
    for (size_t i = 0; i < n; i++) {
        sb_parser_feed(&s->bus_parser, tx[i]);
        while (sb_parser_poll(&s->bus_parser, &pk)) {
            for (int j = 0; j < ARMFW_N_JOINTS; j++) {
                uint8_t reply[SB_MAX_PACKET];
                if (!s->estop_closed) continue; /* servo power is cut: nobody answers */
                size_t r = fake_servo_handle(&s->servo[j], &pk, reply, sizeof reply);
                if (r) rx_append(s, reply, r);
            }
        }
    }
}

static size_t h_servo_read(void *ctx, uint8_t *buf, size_t cap) {
    sim_t *s = (sim_t *)ctx;
    size_t n = s->servo_rx_len < cap ? s->servo_rx_len : cap;
    memcpy(buf, s->servo_rx, n);
    memmove(s->servo_rx, &s->servo_rx[n], s->servo_rx_len - n);
    s->servo_rx_len -= (uint32_t)n;
    s->last_rx_bytes += (uint32_t)n;
    return n;
}

static bool h_estop(void *ctx) { return ((sim_t *)ctx)->estop_closed; }
static uint16_t h_bus_mv(void *ctx) { return sim_bus_mv((sim_t *)ctx); }
static int16_t h_bus_ma(void *ctx) { return sim_bus_ma((sim_t *)ctx); }

int16_t sim_bus_ma(const sim_t *s) {
    if (!s->estop_closed) return 0; /* the hardware loop cuts servo power */
    float sum = 0.0f;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) sum += armfw_fabsf(s->servo[j].current_ma);
    return armfw_f2i16(sum);
}

uint16_t sim_bus_mv(const sim_t *s) {
    if (!s->estop_closed) return 0;
    float v = s->supply_v - s->wiring_ohm * (float)sim_bus_ma(s) * 1e-3f;
    return (uint16_t)armfw_clampi((int32_t)(v * 1000.0f + 0.5f), 0, 65535);
}

/* ---------------- sim ---------------- */

void sim_init(sim_t *s, uint32_t start_us) {
    memset(s, 0, sizeof(*s));
    s->now_us = start_us;
    s->estop_closed = true;
    s->supply_v = 12.0f;
    s->wiring_ohm = 0.05f;
    s->rng = 0x12345678u;
    fifo_init(&s->host_in, SIM_HOST_IN_CAP);
    fifo_init(&s->host_out, SIM_HOST_OUT_CAP);
    sb_parser_init(&s->bus_parser);
    /* Arm starts resting near its zero pose; gripper half open. */
    static const float start_ticks[ARMFW_N_JOINTS] = {2048, 2048, 2048, 2048, 2048, 2570};
    for (int j = 0; j < ARMFW_N_JOINTS; j++) fake_servo_init(&s->servo[j], (uint8_t)(j + 1), start_ticks[j]);
    s->servo[1].hold_ma = 450.0f; /* shoulder carries the most gravity load */
    s->servo[2].hold_ma = 300.0f;
    s->hal.ctx = s;
    s->hal.time_us = h_time_us;
    s->hal.host_read = h_host_read;
    s->hal.host_write = h_host_write;
    s->hal.servo_txrx = h_servo_txrx;
    s->hal.servo_read = h_servo_read;
    s->hal.estop_loop_closed = h_estop;
    s->hal.bus_mv = h_bus_mv;
    s->hal.bus_ma = h_bus_ma;
    control_init(&s->ctl, &s->hal);
}

void sim_step(sim_t *s) {
    uint32_t dt_us = CONTROL_PERIOD_US;
    if (s->jitter_us) {
        s->rng = s->rng * 1664525u + 1013904223u;
        dt_us = CONTROL_PERIOD_US - s->jitter_us + (s->rng >> 8) % (2u * s->jitter_us + 1u);
    }
    s->now_us += dt_us;
    float dt = (float)dt_us * 1e-6f;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        s->servo[j].voltage_v = (float)sim_bus_mv(s) * 1e-3f;
        if (!s->estop_closed) s->servo[j].torque = false; /* power cut */
        fake_servo_update(&s->servo[j], dt);
    }
    s->last_rx_bytes = 0;
    control_step(&s->ctl);
}

void sim_run(sim_t *s, uint32_t steps) {
    for (uint32_t i = 0; i < steps; i++) sim_step(s);
}

size_t sim_host_send(sim_t *s, const uint8_t *data, size_t n) { return fifo_write(&s->host_in, data, n); }

size_t sim_host_recv(sim_t *s, uint8_t *buf, size_t cap) { return fifo_read(&s->host_out, buf, cap); }
