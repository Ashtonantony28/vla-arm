#include "armfw/config.h"

#include "armfw/proto.h"

/* From spec/kinematics.yaml, converted to mrad and mrad/s. Joint order: base_yaw, shoulder_pitch,
 * elbow_pitch, wrist_pitch, wrist_roll, gripper. */
static const int16_t k_lim_min[ARMFW_N_JOINTS] = {-2600, -1800, -2700, -1900, -3000, 0};
static const int16_t k_lim_max[ARMFW_N_JOINTS] = {2600, 750, 2300, 1900, 3000, 1600};
static const int16_t k_vmax[ARMFW_N_JOINTS] = {3000, 2500, 3000, 3500, 4000, 4000};

void config_defaults(armfw_config_t *c) {
    for (int i = 0; i < ARMFW_N_JOINTS; i++) {
        joint_cfg_t *j = &c->joint[i];
        j->offset_ticks = 0;
        j->direction = 1;
        j->limit_min_mrad = k_lim_min[i];
        j->limit_max_mrad = k_lim_max[i];
        j->vmax_mrad_s = k_vmax[i];
        j->current_limit_ma = CFG_DEFAULT_CURRENT_LIMIT_MA;
        j->servo_id = (uint8_t)(i + 1);
    }
    c->wd_hold_ms = CFG_DEFAULT_WD_HOLD_MS;
    c->wd_off_ms = CFG_DEFAULT_WD_OFF_MS;
}

int config_apply(armfw_config_t *c, uint8_t joint, uint8_t key, int32_t value, bool torque_on) {
    bool global_key = (key == PROTO_CFG_WD_HOLD_MS || key == PROTO_CFG_WD_OFF_MS);
    bool joint_key = (key >= PROTO_CFG_OFFSET_TICKS && key <= PROTO_CFG_CURRENT_LIMIT);
    if (!global_key && !joint_key) return PROTO_ERR_UNKNOWN_KEY;

    if (global_key) {
        if (joint != PROTO_CFG_JOINT_GLOBAL) return PROTO_ERR_BAD_VALUE;
        if (key == PROTO_CFG_WD_HOLD_MS) {
            if (value < 1 || value >= (int32_t)c->wd_off_ms) return PROTO_ERR_BAD_VALUE;
            c->wd_hold_ms = (uint16_t)value;
        } else {
            if (value <= (int32_t)c->wd_hold_ms || value > 60000) return PROTO_ERR_BAD_VALUE;
            c->wd_off_ms = (uint16_t)value;
        }
        return 0;
    }

    if (joint >= ARMFW_N_JOINTS) return PROTO_ERR_BAD_VALUE;
    joint_cfg_t *j = &c->joint[joint];
    switch (key) {
    case PROTO_CFG_OFFSET_TICKS:
        if (torque_on) return PROTO_ERR_BAD_VALUE;
        if (value < -(CFG_TICKS_PER_REV - 1) || value > CFG_TICKS_PER_REV - 1) return PROTO_ERR_BAD_VALUE;
        j->offset_ticks = value;
        return 0;
    case PROTO_CFG_DIRECTION:
        if (torque_on) return PROTO_ERR_BAD_VALUE;
        if (value != 1 && value != -1) return PROTO_ERR_BAD_VALUE;
        j->direction = (int8_t)value;
        return 0;
    case PROTO_CFG_LIMIT_MIN:
        if (value < -32767 || value > 32767 || value >= j->limit_max_mrad) return PROTO_ERR_BAD_VALUE;
        j->limit_min_mrad = (int16_t)value;
        return 0;
    case PROTO_CFG_LIMIT_MAX:
        if (value < -32767 || value > 32767 || value <= j->limit_min_mrad) return PROTO_ERR_BAD_VALUE;
        j->limit_max_mrad = (int16_t)value;
        return 0;
    case PROTO_CFG_VMAX:
        if (value < 1 || value > 32767) return PROTO_ERR_BAD_VALUE;
        j->vmax_mrad_s = (int16_t)value;
        return 0;
    case PROTO_CFG_CURRENT_LIMIT:
        if (value < 1 || value > 32767) return PROTO_ERR_BAD_VALUE;
        j->current_limit_ma = (int16_t)value;
        return 0;
    default:
        return PROTO_ERR_UNKNOWN_KEY;
    }
}

uint16_t config_mrad_to_ticks(const joint_cfg_t *j, float mrad) {
    float t = (float)j->direction * mrad * CFG_TICKS_PER_MRAD;
    float base = (float)(CFG_CENTER_TICK + j->offset_ticks);
    float v = base + t;
    v = armfw_clampf(v, 0.0f, (float)CFG_TICK_MAX);
    return (uint16_t)(v + 0.5f);
}

float config_ticks_to_mrad(const joint_cfg_t *j, uint16_t ticks) {
    int32_t d = (int32_t)ticks - CFG_CENTER_TICK - j->offset_ticks;
    return (float)j->direction * (float)d * CFG_MRAD_PER_TICK;
}

float config_tickrate_to_mrad_s(const joint_cfg_t *j, int32_t ticks_per_s) {
    return (float)j->direction * (float)ticks_per_s * CFG_MRAD_PER_TICK;
}
