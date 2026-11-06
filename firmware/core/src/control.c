#include "armfw/control.h"

#include <string.h>

#include "armfw/crc16.h"

/* ---------------- host output ---------------- */

static void send_frame(control_t *c, uint8_t type, const uint8_t *payload, size_t len) {
    uint8_t buf[PROTO_MAX_FRAME];
    size_t n = proto_encode(buf, sizeof buf, type, c->tx_seq++, payload, len);
    if (n == 0) return;
    size_t w = c->hal->host_write(c->hal->ctx, buf, n);
    if (w != n) c->stats.host_tx_dropped++;
}

static void send_ack(control_t *c, const proto_frame_t *f) {
    proto_ack_t a = {f->seq, f->type};
    uint8_t pl[PROTO_LEN_ACK];
    proto_pack_ack(&a, pl);
    send_frame(c, PROTO_T_ACK, pl, sizeof pl);
    c->stats.acks++;
}

static void send_nack(control_t *c, const proto_frame_t *f, int err) {
    proto_nack_t a = {f->seq, f->type, (uint8_t)err};
    uint8_t pl[PROTO_LEN_NACK];
    proto_pack_nack(&a, pl);
    send_frame(c, PROTO_T_NACK, pl, sizeof pl);
    c->stats.nacks++;
}

static void reply(control_t *c, const proto_frame_t *f, int err) {
    if (err)
        send_nack(c, f, err);
    else
        send_ack(c, f);
}

/* ---------------- helpers ---------------- */

static void build_limits(const control_t *c, traj_limits_t *l) {
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        l->lim_min[j] = (float)c->cfg.joint[j].limit_min_mrad;
        l->lim_max[j] = (float)c->cfg.joint[j].limit_max_mrad;
        l->vmax[j] = (float)c->cfg.joint[j].vmax_mrad_s;
    }
}

static bool all_meas_valid(const control_t *c) {
    for (int j = 0; j < ARMFW_N_JOINTS; j++)
        if (!c->meas[j].valid) return false;
    return true;
}

/* Command follows the measured position (torque off), so enabling torque causes no jump. */
static void follow_measured(control_t *c) {
    float pos[ARMFW_N_JOINTS];
    for (int j = 0; j < ARMFW_N_JOINTS; j++)
        pos[j] = c->meas[j].valid ? c->meas[j].pos_mrad : c->traj.cmd[j].p;
    traj_reset(&c->traj, pos);
}

uint16_t control_cmd_pos_crc(const control_t *c) {
    uint8_t b[2 * ARMFW_N_JOINTS];
    for (int j = 0; j < ARMFW_N_JOINTS; j++) armfw_put_i16(&b[2 * j], armfw_f2i16(c->traj.cmd[j].p));
    return crc16_ccitt_false(b, sizeof b);
}

/* ---------------- host command handling ---------------- */

static int handle_set_targets(control_t *c, const proto_frame_t *f) {
    proto_set_targets_t m;
    proto_unpack_set_targets(f->payload, f->len, &m);
    uint32_t now = c->hal->time_us(c->hal->ctx);
    if (!(m.flags & PROTO_FLAG_TORQUE)) {
        /* Torque-off is always executed, whatever the other fields hold (targets and move_ms are
         * ignored); reserved flag bits still earn a NACK so host bugs are visible. */
        safety_request_torque(&c->safety, false, now);
        return (m.flags & (uint8_t)~PROTO_FLAG_TORQUE) ? PROTO_ERR_BAD_VALUE : 0;
    }
    if (m.flags & (uint8_t)~PROTO_FLAG_TORQUE) return PROTO_ERR_BAD_VALUE;
    if (m.move_ms < 1 || m.move_ms > 5000) return PROTO_ERR_BAD_VALUE;
    bool was_on = safety_torque_on(&c->safety);
    if (c->safety.mode == MODE_FAULT) return PROTO_ERR_FAULTED;
    if (!was_on && !all_meas_valid(c)) return PROTO_ERR_FAULTED; /* no feedback yet */
    int r = safety_request_torque(&c->safety, true, now);
    if (r) return r;
    if (!was_on) follow_measured(c);

    traj_limits_t lim;
    build_limits(c, &lim);
    float tgt[ARMFW_N_JOINTS];
    c->clamped_mask = 0;
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        tgt[j] = (float)m.pos[j];
        if (tgt[j] < lim.lim_min[j] || tgt[j] > lim.lim_max[j]) c->clamped_mask |= (uint8_t)(1u << j);
    }
    c->last_plan = traj_replan(&c->traj, tgt, (float)m.move_ms * 1e-3f, &lim);
    c->stats.replans++;
    safety_set_limit_clamp(&c->safety, c->last_plan.pos_clamped || c->last_plan.stretched ||
                                           c->last_plan.overshoot);
    return 0;
}

static void handle_frame(control_t *c, const proto_frame_t *f) {
    uint32_t now = c->hal->time_us(c->hal->ctx);
    c->stats.host_frames++;
    c->last_host_seq = f->seq;
    safety_host_frame(&c->safety, now); /* any well-framed host frame feeds the watchdog */

    if (!proto_is_host_type(f->type)) {
        send_nack(c, f, PROTO_ERR_UNKNOWN_TYPE);
        return;
    }
    if (proto_expected_len(f->type) != (int)f->len) {
        send_nack(c, f, PROTO_ERR_BAD_LEN);
        return;
    }
    switch (f->type) {
    case PROTO_T_PING: {
        proto_pong_t p = {ARMFW_FW_VERSION, c->steps, PROTO_VER};
        uint8_t pl[PROTO_LEN_PONG];
        proto_pack_pong(&p, pl);
        send_frame(c, PROTO_T_PONG, pl, sizeof pl);
        break;
    }
    case PROTO_T_SET_TARGETS:
        reply(c, f, handle_set_targets(c, f));
        break;
    case PROTO_T_CONFIG: {
        proto_config_t m;
        proto_unpack_config(f->payload, f->len, &m);
        reply(c, f, config_apply(&c->cfg, m.joint, m.key, m.value, safety_torque_on(&c->safety)));
        break;
    }
    case PROTO_T_ESTOP:
        safety_estop_soft(&c->safety);
        send_ack(c, f);
        break;
    case PROTO_T_CLEAR_FAULT: {
        proto_clear_fault_t m;
        proto_unpack_clear_fault(f->payload, f->len, &m);
        reply(c, f, safety_clear(&c->safety, m.mask));
        break;
    }
    case PROTO_T_SET_STREAM: {
        proto_set_stream_t m;
        proto_unpack_set_stream(f->payload, f->len, &m);
        if (m.period_us != 0 && m.period_us < CONTROL_MIN_STREAM_US) {
            send_nack(c, f, PROTO_ERR_BAD_VALUE);
        } else {
            c->stream_period_us = m.period_us;
            c->stream_accum_us = 0;
            send_ack(c, f);
        }
        break;
    }
    default:
        send_nack(c, f, PROTO_ERR_UNKNOWN_TYPE);
        break;
    }
}

/* ---------------- servo bus ---------------- */

static void ingest_servo(control_t *c) {
    uint8_t buf[64];
    int pending = c->sched.pending;
    bool answered = false;
    size_t total = 0;
    memset(c->fresh, 0, sizeof c->fresh);
    for (;;) {
        size_t n = c->hal->servo_read(c->hal->ctx, buf, sizeof buf);
        if (n == 0) break;
        total += n;
        for (size_t i = 0; i < n; i++) {
            sb_packet_t pk;
            sb_parser_feed(&c->sb_parser, buf[i]);
            while (sb_parser_poll(&c->sb_parser, &pk)) {
                /* Only the reply to last cycle's READ is accepted; our own TX echo (SYNC_WRITE to
                 * 0xFE, READ instruction with 2 params) is ignored by id/length. */
                if (pending < 0 || answered) continue;
                if (pk.id != c->cfg.joint[pending].servo_id || pk.nparams != SB_FEEDBACK_LEN) continue;
                sb_feedback_t fb;
                if (!sb_decode_feedback(pk.params, pk.nparams, &fb)) continue;
                joint_meas_t *m = &c->meas[pending];
                const joint_cfg_t *jc = &c->cfg.joint[pending];
                m->valid = true;
                m->pos_mrad = config_ticks_to_mrad(jc, fb.pos_ticks);
                m->vel_mrad_s = config_tickrate_to_mrad_s(jc, fb.speed_ticks_s);
                m->current_ma = fb.current_ma;
                m->temp_c = fb.temp_c;
                m->voltage_dv = fb.voltage_dv;
                m->servo_error = pk.code;
                answered = true;
            }
        }
        if (total >= CONTROL_MAX_RX_PER_STEP) break;
    }
    if (pending >= 0) {
        sb_sched_resolve(&c->sched, answered);
        if (answered) {
            c->fresh[pending] = true;
            c->stats.servo_replies++;
        } else {
            c->stats.servo_misses++;
        }
    }
}

static void emit_servo_tx(control_t *c) {
    uint8_t tx[160];
    size_t n = 0;
    uint8_t ids[ARMFW_N_JOINTS];
    for (int j = 0; j < ARMFW_N_JOINTS; j++) ids[j] = c->cfg.joint[j].servo_id;
    bool torque = safety_torque_on(&c->safety);
    bool refresh = (c->steps % CONTROL_TORQUE_REFRESH_MS) == 0;

    /* torque off goes first; torque on goes after the goal write */
    if (!torque && (c->servo_torque || refresh))
        n += sb_encode_sync_torque(&tx[n], sizeof tx - n, ids, ARMFW_N_JOINTS, false);
    if (torque) {
        uint16_t ticks[ARMFW_N_JOINTS];
        for (int j = 0; j < ARMFW_N_JOINTS; j++)
            ticks[j] = config_mrad_to_ticks(&c->cfg.joint[j], c->traj.cmd[j].p);
        n += sb_encode_sync_goal(&tx[n], sizeof tx - n, ids, ticks, ARMFW_N_JOINTS);
        if (!c->servo_torque || refresh)
            n += sb_encode_sync_torque(&tx[n], sizeof tx - n, ids, ARMFW_N_JOINTS, true);
    }
    c->servo_torque = torque;
    uint8_t poll = sb_sched_next(&c->sched);
    n += sb_encode_read(&tx[n], sizeof tx - n, ids[poll], SB_FEEDBACK_ADDR, SB_FEEDBACK_LEN);
    c->hal->servo_txrx(c->hal->ctx, tx, n);
}

/* ---------------- STATE ---------------- */

void control_fill_state(const control_t *c, proto_state_t *st) {
    memset(st, 0, sizeof(*st));
    st->t_us = c->t_prev_us;
    st->fault_flags = c->safety.faults;
    st->mode = (uint8_t)c->safety.mode;
    st->last_host_seq = c->last_host_seq;
    bool torque = safety_torque_on(&c->safety);
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        const joint_meas_t *m = &c->meas[j];
        proto_joint_state_t *js = &st->joint[j];
        uint8_t bit = (uint8_t)(1u << j);
        js->pos_mrad = armfw_f2i16(m->valid ? m->pos_mrad : c->traj.cmd[j].p);
        js->vel_mrad_s = armfw_f2i16(m->vel_mrad_s);
        js->current_ma = m->current_ma;
        js->temp_c = m->temp_c;
        uint8_t s = 0;
        if (torque) s |= JS_TORQUE_ON;
        if (m->valid && c->sched.miss[j] == 0) s |= JS_FEEDBACK_OK;
        if (c->clamped_mask & bit) s |= JS_TARGET_CLAMPED;
        if (c->safety.joint_overcurrent & bit) s |= JS_OVERCURRENT;
        if (c->safety.joint_overtemp & bit) s |= JS_OVERTEMP;
        if (c->safety.joint_comms & bit) s |= JS_COMMS_FAULT;
        if (c->traj.moving && torque) s |= JS_MOVING;
        if (m->servo_error) s |= JS_SERVO_ERROR;
        js->status = s;
    }
    st->bus_mv = c->hal->bus_mv(c->hal->ctx);
    st->bus_ma = c->hal->bus_ma(c->hal->ctx);
    st->loop_jitter_max_us = c->jitter_max_us;
    st->loop_overruns = c->overruns;
    st->cmd_pos_crc = control_cmd_pos_crc(c);
}

static void send_state(control_t *c) {
    proto_state_t st;
    control_fill_state(c, &st);
    uint8_t pl[PROTO_LEN_STATE];
    proto_pack_state(&st, pl);
    send_frame(c, PROTO_T_STATE, pl, sizeof pl);
    c->stats.states++;
    c->jitter_max_us = 0;
}

/* ---------------- init / step ---------------- */

void control_init(control_t *c, const hal_port_t *hal) {
    memset(c, 0, sizeof(*c));
    c->hal = hal;
    config_defaults(&c->cfg);
    uint32_t now = hal->time_us(hal->ctx);
    safety_init(&c->safety, now);
    proto_parser_init(&c->host_parser);
    sb_parser_init(&c->sb_parser);
    sb_sched_init(&c->sched, ARMFW_N_JOINTS);
    float zero[ARMFW_N_JOINTS] = {0};
    traj_reset(&c->traj, zero);
    c->stream_period_us = CONTROL_DEFAULT_STREAM_US;
    c->t_prev_us = now;
    c->servo_torque = true; /* forces a torque-off write on the first step */
    c->prev_mode = MODE_IDLE;
}

void control_step(control_t *c) {
    const hal_port_t *h = c->hal;
    uint32_t now = h->time_us(h->ctx);

    /* loop timing */
    if (c->steps > 0) {
        uint32_t period = now - c->t_prev_us;
        uint32_t jit = period > CONTROL_PERIOD_US ? period - CONTROL_PERIOD_US : CONTROL_PERIOD_US - period;
        if (jit > 0xFFFFu) jit = 0xFFFFu;
        if (jit > c->jitter_max_us) c->jitter_max_us = (uint16_t)jit;
        if (period >= 2u * CONTROL_PERIOD_US) c->overruns++;
    }
    c->t_prev_us = now;
    c->steps++;

    /* 1. servo replies to the previous cycle's READ */
    ingest_servo(c);

    /* 2. host frames */
    {
        uint8_t buf[64];
        size_t total = 0;
        proto_frame_t f;
        for (;;) {
            size_t n = h->host_read(h->ctx, buf, sizeof buf);
            if (n == 0) break;
            total += n;
            for (size_t i = 0; i < n; i++) {
                proto_parser_feed(&c->host_parser, buf[i]);
                while (proto_parser_poll(&c->host_parser, &f)) handle_frame(c, &f);
            }
            if (total >= CONTROL_MAX_RX_PER_STEP) break;
        }
    }

    /* 3. safety */
    safety_inputs_t in;
    memset(&in, 0, sizeof in);
    in.now_us = now;
    in.estop_loop_closed = h->estop_loop_closed(h->ctx);
    in.bus_mv = h->bus_mv(h->ctx);
    in.bus_ma = h->bus_ma(h->ctx);
    for (int j = 0; j < ARMFW_N_JOINTS; j++) {
        in.fresh[j] = c->fresh[j];
        in.current_ma[j] = c->meas[j].current_ma;
        in.temp_c[j] = c->meas[j].temp_c;
        in.comms_miss[j] = c->sched.miss[j];
    }
    safety_update(&c->safety, &c->cfg, &in);

    /* 4. motion */
    traj_limits_t lim;
    build_limits(c, &lim);
    switch (c->safety.mode) {
    case MODE_ACTIVE:
        traj_step(&c->traj, CONTROL_PERIOD_US, &lim);
        if (c->traj.saturated) safety_set_limit_clamp(&c->safety, true);
        break;
    case MODE_HOLD:
        if (c->prev_mode != MODE_HOLD) traj_freeze(&c->traj);
        traj_step(&c->traj, CONTROL_PERIOD_US, &lim);
        break;
    case MODE_IDLE:
    case MODE_FAULT:
    default:
        follow_measured(c);
        break;
    }
    c->prev_mode = c->safety.mode;

    /* 5. servo bus: goals (+ torque state) and one round-robin feedback READ */
    emit_servo_tx(c);

    /* 6. STATE stream */
    if (c->stream_period_us) {
        c->stream_accum_us += CONTROL_PERIOD_US;
        if (c->stream_accum_us >= c->stream_period_us) {
            c->stream_accum_us -= c->stream_period_us;
            send_state(c);
        }
    }

    uint32_t exec = h->time_us(h->ctx) - now;
    c->last_step_exec_us = exec;
    if (exec > CONTROL_PERIOD_US) c->overruns++;
}
