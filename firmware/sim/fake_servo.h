/* Fake STS3215-class bus servo for host simulation.
 * Position follows the goal with a first-order response (time constant tau) limited by a maximum
 * speed; current = idle + gravity/holding + speed and acceleration terms, signed by the effort. */
#ifndef ARMFW_FAKE_SERVO_H
#define ARMFW_FAKE_SERVO_H

#include "armfw/common.h"
#include "armfw/servo_bus.h"

typedef struct {
    uint8_t id;
    bool torque;
    float pos_ticks;
    float vel_ticks_s;
    float acc_ticks_s2;
    float goal_ticks;
    float current_ma;
    /* model parameters */
    float tau_s;              /* first-order time constant */
    float max_speed_ticks_s;  /* no-load speed */
    float idle_ma;
    float hold_ma;            /* gravity / holding current while torque is on */
    float ma_per_tick_s;      /* speed term */
    float ma_per_tick_s2;     /* acceleration (inertia) term */
    float temp_c;
    float voltage_v;
    /* fault injection */
    bool silent;              /* never answers */
    bool inject_current;      /* report current_override_ma instead of the model */
    int16_t current_override_ma;
    uint8_t error_byte;       /* error byte in status packets */
    uint32_t reads_answered;
    uint32_t goal_writes;
} fake_servo_t;

void fake_servo_init(fake_servo_t *s, uint8_t id, float pos_ticks);
void fake_servo_update(fake_servo_t *s, float dt);
/* Handle one instruction packet addressed to this servo (or broadcast). Writes the status reply
 * into reply (if any) and returns its length. */
size_t fake_servo_handle(fake_servo_t *s, const sb_packet_t *pk, uint8_t *reply, size_t cap);
/* Fill the 15-byte feedback block (registers 0x38..0x46). */
void fake_servo_feedback(const fake_servo_t *s, uint8_t out[SB_FEEDBACK_LEN]);

#endif
