#include "armfw/safety.h"

#include <string.h>

/* Latched faults force FAULT. Clearing them does not leave FAULT by itself: the mode stays FAULT
 * (fault_flags may read 0) until a SET_TARGETS - torque on -> ACTIVE, torque off -> IDLE. */
static void resolve_mode(safety_t *s) {
    if (s->faults & FAULT_TORQUE_OFF_MASK) s->mode = MODE_FAULT;
}

void safety_init(safety_t *s, uint32_t now_us) {
    memset(s, 0, sizeof(*s));
    s->mode = MODE_IDLE;
    s->last_host_us = now_us;
}

void safety_host_frame(safety_t *s, uint32_t now_us) {
    s->last_host_us = now_us;
    if (s->mode == MODE_HOLD) s->mode = MODE_ACTIVE;
}

int safety_request_torque(safety_t *s, bool on, uint32_t now_us) {
    if (on) {
        if (s->faults & FAULT_TORQUE_OFF_MASK) return PROTO_ERR_FAULTED;
        s->mode = MODE_ACTIVE;
        s->last_host_us = now_us;
    } else if (s->mode != MODE_FAULT || !(s->faults & FAULT_TORQUE_OFF_MASK)) {
        s->mode = MODE_IDLE;
    }
    return 0;
}

void safety_estop_soft(safety_t *s) {
    s->faults |= FAULT_ESTOP_SOFT;
    resolve_mode(s);
}

void safety_clear(safety_t *s, uint16_t mask) {
    mask &= FAULT_DEFINED_MASK; /* undefined bits are ignored */
    uint16_t clearable = (uint16_t)(mask & ~s->live);
    s->faults = (uint16_t)(s->faults & ~clearable);
    if (clearable & FAULT_OVERCURRENT) {
        memset(s->oc_count, 0, sizeof(s->oc_count));
        s->bus_oc_ms = 0;
    }
    resolve_mode(s);
}

void safety_set_limit_clamp(safety_t *s, bool on) {
    if (on)
        s->faults |= FAULT_LIMIT_CLAMP;
    else
        s->faults = (uint16_t)(s->faults & ~FAULT_LIMIT_CLAMP);
}

static void set_live(safety_t *s, uint16_t bit, bool present) {
    if (present) {
        s->live |= bit;
    } else {
        s->live = (uint16_t)(s->live & ~bit);
    }
}

void safety_update(safety_t *s, const armfw_config_t *cfg, const safety_inputs_t *in) {
    /* Hardware e-stop loop */
    if (!in->estop_loop_closed) s->faults |= FAULT_ESTOP_HW;
    set_live(s, FAULT_ESTOP_HW, !in->estop_loop_closed);

    /* Per-joint servo checks, evaluated on fresh samples only */
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        uint8_t bit = (uint8_t)(1u << j);
        if (in->fresh[j]) {
            int32_t cur = in->current_ma[j] < 0 ? -(int32_t)in->current_ma[j] : in->current_ma[j];
            if (cur > cfg->joint[j].current_limit_ma) {
                if (s->oc_count[j] < 255) s->oc_count[j]++;
                s->joint_overcurrent |= bit;
                if (s->oc_count[j] >= SAFETY_OC_SAMPLES) s->faults |= FAULT_OVERCURRENT;
            } else {
                s->oc_count[j] = 0;
                s->joint_overcurrent = (uint8_t)(s->joint_overcurrent & ~bit);
            }
            if (in->temp_c[j] > SAFETY_OVERTEMP_C) {
                s->joint_overtemp |= bit;
                s->faults |= FAULT_OVERTEMP;
            } else {
                s->joint_overtemp = (uint8_t)(s->joint_overtemp & ~bit);
            }
        }
        if (in->comms_miss[j] >= SAFETY_COMMS_MISSES) {
            s->joint_comms |= bit;
            s->faults |= FAULT_SERVO_COMMS;
        } else {
            s->joint_comms = (uint8_t)(s->joint_comms & ~bit);
        }
    }

    /* Total servo-rail current from the board's shunt */
    int32_t bus = in->bus_ma < 0 ? -(int32_t)in->bus_ma : in->bus_ma;
    if (bus > SAFETY_BUS_OC_MA) {
        if (s->bus_oc_ms < 0xFFFFu) s->bus_oc_ms++;
        if (s->bus_oc_ms >= SAFETY_BUS_OC_MS) s->faults |= FAULT_OVERCURRENT;
    } else {
        s->bus_oc_ms = 0;
    }
    set_live(s, FAULT_OVERCURRENT, s->joint_overcurrent != 0 || s->bus_oc_ms != 0);
    /* Board temperature (NTC between the servo-rail switch and the shunt) */
    if (in->board_temp_valid && in->board_temp_dc > SAFETY_BOARD_OT_DC) {
        if (s->board_hot_ms < 0xFFFFu) s->board_hot_ms++;
        if (s->board_hot_ms >= SAFETY_BOARD_OT_MS) s->faults |= FAULT_OVERTEMP;
    } else {
        s->board_hot_ms = 0;
    }
    s->board_overtemp = in->board_temp_valid && in->board_temp_dc >= SAFETY_BOARD_OT_LIVE_DC;
    set_live(s, FAULT_OVERTEMP, s->joint_overtemp != 0 || s->board_overtemp);
    set_live(s, FAULT_SERVO_COMMS, s->joint_comms != 0);

    /* Bus undervoltage, live with hysteresis */
    if (in->bus_mv < SAFETY_UNDERVOLT_MV) {
        s->uv_ok_ms = 0;
        if (s->uv_low_ms < 0xFFFFu) s->uv_low_ms++;
        if (s->uv_low_ms >= SAFETY_UNDERVOLT_SET_MS) s->faults |= FAULT_BUS_UNDERVOLT;
    } else if (in->bus_mv >= SAFETY_UNDERVOLT_RECOVER_MV) {
        s->uv_low_ms = 0;
        if (s->uv_ok_ms < 0xFFFFu) s->uv_ok_ms++;
        if (s->uv_ok_ms >= SAFETY_UNDERVOLT_CLEAR_MS)
            s->faults = (uint16_t)(s->faults & ~FAULT_BUS_UNDERVOLT);
    } else {
        s->uv_low_ms = 0;
        s->uv_ok_ms = 0;
    }
    set_live(s, FAULT_BUS_UNDERVOLT, (s->faults & FAULT_BUS_UNDERVOLT) != 0);

    /* Watchdog tiers, armed only while torque is on */
    if (safety_torque_on(s)) {
        uint32_t silent = in->now_us - s->last_host_us; /* wrap-safe */
        if (silent > (uint32_t)cfg->wd_off_ms * 1000u) {
            s->faults |= FAULT_WATCHDOG;
        } else if (silent > (uint32_t)cfg->wd_hold_ms * 1000u && s->mode == MODE_ACTIVE) {
            s->mode = MODE_HOLD;
        }
    }

    resolve_mode(s);
}
