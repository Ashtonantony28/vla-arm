/* The 1 kHz control step: host frames in, trajectories advanced, servo bus out, servo replies in,
 * safety evaluated, STATE frames out at the stream period, ACK/NACK/PONG answers. */
#ifndef ARMFW_CONTROL_H
#define ARMFW_CONTROL_H

#include "armfw/analog.h"
#include "armfw/calstore.h"
#include "armfw/common.h"
#include "armfw/config.h"
#include "armfw/hal_port.h"
#include "armfw/proto.h"
#include "armfw/safety.h"
#include "armfw/servo_bus.h"
#include "armfw/traj.h"

#define CONTROL_PERIOD_US 1000u
#define CONTROL_DEFAULT_STREAM_US 5000u
#define CONTROL_MIN_STREAM_US 1000u
#define CONTROL_TORQUE_REFRESH_MS 100u /* re-send the torque state every 100 ms */
#define CONTROL_MAX_RX_PER_STEP 1024u  /* bytes per link per step, bounds step time */

/* joint_status bits in STATE (the spec does not define them; see README) */
#define JS_TORQUE_ON (1u << 0)
#define JS_FEEDBACK_OK (1u << 1)
#define JS_TARGET_CLAMPED (1u << 2)
#define JS_OVERCURRENT (1u << 3)
#define JS_OVERTEMP (1u << 4)
#define JS_COMMS_FAULT (1u << 5)
#define JS_MOVING (1u << 6)
#define JS_SERVO_ERROR (1u << 7)

typedef struct {
    bool valid;
    float pos_mrad;
    float vel_mrad_s;
    int16_t current_ma;
    uint8_t temp_c;
    uint8_t voltage_dv;
    uint8_t servo_error; /* error byte of the last status packet */
} joint_meas_t;

typedef struct {
    uint32_t host_frames;
    uint32_t acks;
    uint32_t nacks;
    uint32_t states;
    uint32_t host_tx_dropped;
    uint32_t servo_replies;
    uint32_t servo_misses;
    uint32_t replans;
    uint32_t cal_saves;
    uint32_t cal_save_failures;
} control_stats_t;

typedef struct {
    const hal_port_t *hal;
    armfw_config_t cfg;
    safety_t safety;
    traj_group_t traj;
    traj_plan_result_t last_plan;
    proto_parser_t host_parser;
    sb_parser_t sb_parser;
    sb_sched_t sched;
    joint_meas_t meas[ARMFW_N_JOINTS];
    bool fresh[ARMFW_N_JOINTS];
    uint8_t clamped_mask;
    uint8_t tx_seq;
    uint8_t last_host_seq;
    uint16_t stream_period_us;
    uint32_t stream_accum_us;
    uint32_t steps; /* also uptime in ms */
    uint32_t t_prev_us;
    uint16_t jitter_max_us; /* since the last STATE frame */
    uint16_t overruns;
    uint32_t last_step_exec_us;
    bool servo_torque; /* torque state last sent to the servos */
    armfw_mode_t prev_mode;
    analog_t analog;
    calstore_t store;
    bool cal_from_flash;    /* boot configuration came from a valid flash record */
    bool save_pending;      /* CONFIG 0x20 waiting for the flash write to finish */
    uint8_t save_seq;       /* its SEQ, for the deferred ACK/NACK */
    control_stats_t stats;
} control_t;

/* Initialise; loads the calibration from hal->flash when present (defaults otherwise). */
void control_init(control_t *c, const hal_port_t *hal);
void control_step(control_t *c);

/* Exposed for tests and the sim: CRC of the 12 bytes of current commanded positions. */
uint16_t control_cmd_pos_crc(const control_t *c);
/* Fill a STATE payload struct from the current state (does not reset jitter_max). */
void control_fill_state(const control_t *c, proto_state_t *st);

#endif
