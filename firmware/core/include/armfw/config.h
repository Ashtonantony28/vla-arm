/* Per-joint calibration and limits, plus global watchdog settings. Defaults mirror
 * spec/kinematics.yaml (limits, vmax) and spec/protocol.md (watchdog 100/500 ms). */
#ifndef ARMFW_CONFIG_H
#define ARMFW_CONFIG_H

#include "armfw/common.h"
#include "armfw/proto.h"

#define CFG_TICKS_PER_REV 4096
#define CFG_CENTER_TICK 2048
#define CFG_TICK_MAX 4095
/* mrad per servo tick = 2*pi*1000/4096 */
#define CFG_MRAD_PER_TICK 1.5339807878856412f
#define CFG_TICKS_PER_MRAD 0.6518986469044033f

#define CFG_DEFAULT_CURRENT_LIMIT_MA 2000
#define CFG_DEFAULT_WD_HOLD_MS 100
#define CFG_DEFAULT_WD_OFF_MS 500

/* Proposed protocol additions (global CONFIG keys, joint 0xFF; see firmware/README.md):
 *   0x20 save the current configuration to flash, value must be CFG_COMMIT_MAGIC. ACK is sent
 *        only after the record is written and read back; NACK 4 if there is no flash, a save is
 *        already running, or the write failed.
 *   0x21 restore the built-in defaults in RAM (not in flash: follow with 0x20 to persist), value
 *        must be CFG_COMMIT_MAGIC; refused while torque is on (offsets would jump the arm). */
#define CFG_KEY_SAVE_FLASH 0x20u
#define CFG_KEY_RESTORE_DEFAULTS 0x21u
#define CFG_COMMIT_MAGIC 0x5AFE
/* NACK code for calibration keys while torque is on. The firmware has used 2 since wave 1 (and
 * tests/integration expects 2); spec v1.1 clarification 5 says 4. One place to flip it. */
#define CFG_ERR_TORQUE_ON PROTO_ERR_BAD_VALUE

typedef struct {
    int32_t offset_ticks; /* tick reading at joint zero is CENTER + offset_ticks */
    int8_t direction;     /* +1 or -1: sign of servo ticks per joint mrad */
    int16_t limit_min_mrad;
    int16_t limit_max_mrad;
    int16_t vmax_mrad_s;
    int16_t current_limit_ma;
    uint8_t servo_id; /* bus ID, fixed: joint i -> ID i+1 */
} joint_cfg_t;

typedef struct {
    joint_cfg_t joint[ARMFW_N_JOINTS];
    uint16_t wd_hold_ms;
    uint16_t wd_off_ms;
} armfw_config_t;

void config_defaults(armfw_config_t *c);

/* Apply a CONFIG message. torque_on: true in ACTIVE/HOLD (calibration keys are refused then).
 * Returns 0 on success or a PROTO_ERR_* NACK code. Validation never leaves partial changes. */
int config_apply(armfw_config_t *c, uint8_t joint, uint8_t key, int32_t value, bool torque_on);

/* True if every field passes the same range checks as a CONFIG message would (used to validate
 * records loaded from flash). */
bool config_validate(const armfw_config_t *c);

/* Joint angle (mrad) -> servo goal tick, clamped to 0..4095. */
uint16_t config_mrad_to_ticks(const joint_cfg_t *j, float mrad);
/* Servo position tick -> joint angle (mrad). */
float config_ticks_to_mrad(const joint_cfg_t *j, uint16_t ticks);
/* Servo speed (signed ticks/s) -> joint velocity (mrad/s). */
float config_tickrate_to_mrad_s(const joint_cfg_t *j, int32_t ticks_per_s);

#endif
