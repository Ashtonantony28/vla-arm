/* Per-joint calibration and limits, plus global watchdog settings. Defaults mirror
 * spec/kinematics.yaml (limits, vmax) and spec/protocol.md (watchdog 100/500 ms). */
#ifndef ARMFW_CONFIG_H
#define ARMFW_CONFIG_H

#include "armfw/common.h"

#define CFG_TICKS_PER_REV 4096
#define CFG_CENTER_TICK 2048
#define CFG_TICK_MAX 4095
/* mrad per servo tick = 2*pi*1000/4096 */
#define CFG_MRAD_PER_TICK 1.5339807878856412f
#define CFG_TICKS_PER_MRAD 0.6518986469044033f

#define CFG_DEFAULT_CURRENT_LIMIT_MA 2000
#define CFG_DEFAULT_WD_HOLD_MS 100
#define CFG_DEFAULT_WD_OFF_MS 500

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

/* Joint angle (mrad) -> servo goal tick, clamped to 0..4095. */
uint16_t config_mrad_to_ticks(const joint_cfg_t *j, float mrad);
/* Servo position tick -> joint angle (mrad). */
float config_ticks_to_mrad(const joint_cfg_t *j, uint16_t ticks);
/* Servo speed (signed ticks/s) -> joint velocity (mrad/s). */
float config_tickrate_to_mrad_s(const joint_cfg_t *j, int32_t ticks_per_s);

#endif
