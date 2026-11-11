/* Safety: mode state machine (IDLE / ACTIVE / HOLD / FAULT), fault flags, watchdog tiers.
 *
 * Latched faults (ESTOP_SOFT, ESTOP_HW, WATCHDOG, OVERCURRENT, OVERTEMP, SERVO_COMMS) put the
 * board in FAULT with torque off. BUS_UNDERVOLT is live (not latched) and informational: it is
 * set/cleared with the bus voltage (with hysteresis) and does not change the mode. LIMIT_CLAMP is
 * informational only.
 *
 * Leaving FAULT is two steps: CLEAR_FAULT for every latched bit (a bit whose condition is still
 * present stays set), after which the mode byte stays FAULT with no latched bits, and then
 * SET_TARGETS with torque enable, FAULT -> ACTIVE (torque off instead: FAULT -> IDLE).
 *
 * Watchdog (armed only while torque is on, i.e. ACTIVE or HOLD): host silent > wd_hold_ms:
 * ACTIVE -> HOLD. Host silent > wd_off_ms: latch WATCHDOG -> FAULT. Any valid host frame resets
 * the timer, and in HOLD it returns the board to ACTIVE.
 */
#ifndef ARMFW_SAFETY_H
#define ARMFW_SAFETY_H

#include "armfw/common.h"
#include "armfw/config.h"
#include "armfw/proto.h"

typedef enum {
    MODE_IDLE = PROTO_MODE_IDLE,
    MODE_ACTIVE = PROTO_MODE_ACTIVE,
    MODE_HOLD = PROTO_MODE_HOLD,
    MODE_FAULT = PROTO_MODE_FAULT,
} armfw_mode_t;

#define FAULT_LATCHED_MASK                                                                   \
    (FAULT_ESTOP_SOFT | FAULT_ESTOP_HW | FAULT_WATCHDOG | FAULT_OVERCURRENT | FAULT_OVERTEMP | \
     FAULT_SERVO_COMMS)
#define FAULT_TORQUE_OFF_MASK FAULT_LATCHED_MASK

/* Thresholds (spec values where the spec gives one, otherwise documented choices). */
#define SAFETY_OVERTEMP_C 70             /* spec: > 70 C */
#define SAFETY_COMMS_MISSES 3            /* spec: 3 polls in a row */
#define SAFETY_UNDERVOLT_MV 10500        /* spec: < 10.5 V */
#define SAFETY_UNDERVOLT_RECOVER_MV 10700 /* choice: 200 mV hysteresis */
#define SAFETY_UNDERVOLT_SET_MS 5        /* choice: below threshold for 5 consecutive ms */
#define SAFETY_UNDERVOLT_CLEAR_MS 50     /* choice: above recover level for 50 ms */
#define SAFETY_OC_SAMPLES 8              /* choice: 8 consecutive over-limit reads (~48 ms/joint) */
#define SAFETY_BUS_OC_MA 8000            /* choice: total servo rail current limit (10 A PSU) */
#define SAFETY_BUS_OC_MS 50              /* choice: 50 consecutive ms over the bus limit */
#define SAFETY_BOARD_OT_DC 850           /* choice: board NTC > 85.0 C ... */
#define SAFETY_BOARD_OT_MS 1000          /* ... for 1 s latches OVERTEMP */
#define SAFETY_BOARD_OT_LIVE_DC 800      /* choice: OVERTEMP stays uncleareable while >= 80.0 C */

typedef struct {
    uint32_t now_us;
    bool estop_loop_closed; /* hardware e-stop loop state (true = closed = OK) */
    uint16_t bus_mv;
    int16_t bus_ma;
    int16_t board_temp_dc;  /* board NTC, deci-degC */
    bool board_temp_valid;  /* false: sensor open/short or ADC stale -> no board overtemp check */
    /* Per-joint servo feedback. fresh[j] is true only on the cycle a new reply was decoded. */
    bool fresh[ARMFW_N_JOINTS];
    int16_t current_ma[ARMFW_N_JOINTS];
    uint8_t temp_c[ARMFW_N_JOINTS];
    uint8_t comms_miss[ARMFW_N_JOINTS]; /* consecutive unanswered polls */
} safety_inputs_t;

typedef struct {
    armfw_mode_t mode;
    uint16_t faults;
    uint16_t live; /* bits whose condition is currently present (for CLEAR_FAULT) */
    uint32_t last_host_us;
    uint8_t oc_count[ARMFW_N_JOINTS];
    uint16_t bus_oc_ms;
    uint16_t uv_low_ms;
    uint16_t uv_ok_ms;
    uint16_t board_hot_ms;
    bool board_overtemp; /* live: board NTC at or above SAFETY_BOARD_OT_LIVE_DC */
    uint8_t joint_overcurrent; /* bitmask, live per joint */
    uint8_t joint_overtemp;    /* bitmask, live per joint */
    uint8_t joint_comms;       /* bitmask, live per joint */
} safety_t;

void safety_init(safety_t *s, uint32_t now_us);
/* Any valid host frame: reset watchdog; HOLD -> ACTIVE. */
void safety_host_frame(safety_t *s, uint32_t now_us);
/* SET_TARGETS torque flag. Returns 0, or PROTO_ERR_FAULTED when torque-on is refused because
 * latched faults remain. */
int safety_request_torque(safety_t *s, bool on, uint32_t now_us);
/* ESTOP message: latch ESTOP_SOFT -> FAULT. */
void safety_estop_soft(safety_t *s);
/* CLEAR_FAULT (always ACKed). Clears every requested bit whose condition is gone; bits whose
 * condition persists (loop still open, still hot, ...) and BUS_UNDERVOLT stay set; undefined bits
 * 8-15 are ignored. */
void safety_clear(safety_t *s, uint16_t mask);
/* Set / clear the informational LIMIT_CLAMP flag. */
void safety_set_limit_clamp(safety_t *s, bool on);
/* Evaluate inputs once per control cycle (1 ms). */
void safety_update(safety_t *s, const armfw_config_t *cfg, const safety_inputs_t *in);

static inline bool safety_torque_on(const safety_t *s) {
    return s->mode == MODE_ACTIVE || s->mode == MODE_HOLD;
}

#endif
