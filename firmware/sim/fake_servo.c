#include "fake_servo.h"

#include <string.h>

void fake_servo_init(fake_servo_t *s, uint8_t id, float pos_ticks) {
    memset(s, 0, sizeof(*s));
    s->id = id;
    s->pos_ticks = pos_ticks;
    s->goal_ticks = pos_ticks;
    s->tau_s = 0.030f;
    s->max_speed_ticks_s = 3000.0f; /* ~4.6 rad/s, STS3215 at 12 V */
    s->idle_ma = 20.0f;
    s->hold_ma = 150.0f;
    s->ma_per_tick_s = 0.08f;
    s->ma_per_tick_s2 = 0.002f;
    s->temp_c = 35.0f;
    s->voltage_v = 12.0f;
}

void fake_servo_update(fake_servo_t *s, float dt) {
    float v_new = 0.0f;
    if (s->torque) {
        float err = s->goal_ticks - s->pos_ticks;
        v_new = armfw_clampf(err / s->tau_s, -s->max_speed_ticks_s, s->max_speed_ticks_s);
        /* do not overshoot the goal within one step */
        if (armfw_fabsf(v_new * dt) > armfw_fabsf(err)) v_new = err / dt;
    }
    s->acc_ticks_s2 = (v_new - s->vel_ticks_s) / dt;
    s->vel_ticks_s = v_new;
    s->pos_ticks = armfw_clampf(s->pos_ticks + v_new * dt, 0.0f, 4095.0f);
    if (s->torque) {
        float effort = s->goal_ticks - s->pos_ticks;
        float mag = s->idle_ma + s->hold_ma + s->ma_per_tick_s * armfw_fabsf(s->vel_ticks_s) +
                    s->ma_per_tick_s2 * armfw_fabsf(s->acc_ticks_s2);
        s->current_ma = effort < 0.0f ? -mag : mag;
    } else {
        s->current_ma = s->idle_ma;
    }
}

void fake_servo_feedback(const fake_servo_t *s, uint8_t out[SB_FEEDBACK_LEN]) {
    memset(out, 0, SB_FEEDBACK_LEN);
    int32_t pos = (int32_t)(s->pos_ticks + 0.5f);
    int32_t spd = (int32_t)(s->vel_ticks_s >= 0.0f ? s->vel_ticks_s + 0.5f : s->vel_ticks_s - 0.5f);
    float cur_ma = s->inject_current ? (float)s->current_override_ma : s->current_ma;
    int32_t cur = (int32_t)(cur_ma / SB_CURRENT_MA_PER_LSB + (cur_ma >= 0.0f ? 0.5f : -0.5f));
    int32_t load = (int32_t)(armfw_clampf(cur_ma / 2.7f, -1000.0f, 1000.0f)); /* % of stall, 0.1 % */
    armfw_put_u16(&out[0], (uint16_t)armfw_clampi(pos, 0, 4095));
    armfw_put_u16(&out[2], sb_sign_mag_encode(spd, 15));
    armfw_put_u16(&out[4], sb_sign_mag_encode(load, 10));
    out[6] = (uint8_t)armfw_clampi((int32_t)(s->voltage_v * 10.0f + 0.5f), 0, 255);
    out[7] = (uint8_t)armfw_clampi((int32_t)(s->temp_c + 0.5f), 0, 255);
    out[9] = s->error_byte;
    out[10] = armfw_fabsf(s->vel_ticks_s) > 1.0f ? 1u : 0u;
    armfw_put_u16(&out[13], sb_sign_mag_encode(cur, 15));
}

static void apply_write(fake_servo_t *s, uint8_t addr, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint8_t a = (uint8_t)(addr + i);
        if (a == SB_REG_TORQUE_ENABLE) s->torque = d[i] != 0;
    }
    if (addr <= SB_REG_GOAL_POSITION && addr + n >= (size_t)SB_REG_GOAL_POSITION + 2u) {
        const uint8_t *g = &d[SB_REG_GOAL_POSITION - addr];
        s->goal_ticks = (float)armfw_clampi(sb_sign_mag_decode(armfw_get_u16(g), 15), 0, 4095);
        s->goal_writes++;
    }
}

size_t fake_servo_handle(fake_servo_t *s, const sb_packet_t *pk, uint8_t *reply, size_t cap) {
    bool to_me = pk->id == s->id;
    if (pk->code == SB_INST_SYNC_WRITE && pk->id == SB_BROADCAST_ID && pk->nparams >= 2) {
        uint8_t addr = pk->params[0], len = pk->params[1];
        size_t blk = (size_t)len + 1u;
        for (size_t k = 2; k + blk <= pk->nparams; k += blk)
            if (pk->params[k] == s->id) apply_write(s, addr, &pk->params[k + 1], len);
        return 0; /* broadcast: no reply */
    }
    if (!to_me && pk->id != SB_BROADCAST_ID) return 0;
    size_t r = 0;
    switch (pk->code) {
    case SB_INST_PING:
        if (to_me) r = sb_encode_status(reply, cap, s->id, s->error_byte, NULL, 0);
        break;
    case SB_INST_WRITE:
        if (pk->nparams >= 1) apply_write(s, pk->params[0], &pk->params[1], (size_t)pk->nparams - 1u);
        if (to_me) r = sb_encode_status(reply, cap, s->id, s->error_byte, NULL, 0);
        break;
    case SB_INST_READ:
        if (to_me && pk->nparams == 2 && pk->params[0] == SB_FEEDBACK_ADDR && pk->params[1] == SB_FEEDBACK_LEN) {
            uint8_t fb[SB_FEEDBACK_LEN];
            fake_servo_feedback(s, fb);
            r = sb_encode_status(reply, cap, s->id, s->error_byte, fb, SB_FEEDBACK_LEN);
        }
        break;
    default:
        break;
    }
    if (s->silent) return 0;
    if (r && pk->code == SB_INST_READ) s->reads_answered++;
    return r;
}
