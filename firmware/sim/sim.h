/* Host simulation: the real control core running against a fake HAL and six fake servos, in
 * virtual time (one sim_step = one 1 ms control period). Single-threaded, no allocation. */
#ifndef ARMFW_SIM_H
#define ARMFW_SIM_H

#include "armfw/control.h"
#include "fake_servo.h"

#define SIM_HOST_IN_CAP 16384u
#define SIM_HOST_OUT_CAP 65536u
#define SIM_SERVO_RX_CAP 1024u

typedef struct {
    uint8_t buf[SIM_HOST_OUT_CAP];
    uint32_t head, tail; /* read at head, write at tail, size = tail - head */
    uint32_t cap;
} sim_fifo_t;

typedef struct {
    control_t ctl;
    hal_port_t hal;
    fake_servo_t servo[ARMFW_N_JOINTS];
    uint32_t now_us;
    sim_fifo_t host_in;  /* host -> board */
    sim_fifo_t host_out; /* board -> host */
    uint8_t servo_rx[SIM_SERVO_RX_CAP];
    uint32_t servo_rx_len;
    sb_parser_t bus_parser; /* the servos' view of the bus */
    /* environment / fault injection */
    bool estop_closed;
    float supply_v;
    float wiring_ohm;
    bool echo_tx;          /* half-duplex echo of our own TX into RX */
    uint32_t jitter_us;    /* if nonzero, add pseudo-random timing jitter up to this much */
    uint32_t rng;
    /* observations */
    uint32_t last_tx_bytes;
    uint32_t max_tx_bytes;
    uint32_t last_rx_bytes;
} sim_t;

void sim_init(sim_t *s, uint32_t start_us);
/* Advance virtual time by one period, update servo physics, run one control step. */
void sim_step(sim_t *s);
void sim_run(sim_t *s, uint32_t steps);
/* Bytes from the host to the board. Returns bytes accepted. */
size_t sim_host_send(sim_t *s, const uint8_t *data, size_t n);
/* Bytes from the board to the host. Returns bytes copied. */
size_t sim_host_recv(sim_t *s, uint8_t *buf, size_t cap);
/* Current servo-rail current and voltage as the board ADC would see them. */
int16_t sim_bus_ma(const sim_t *s);
uint16_t sim_bus_mv(const sim_t *s);

#endif
