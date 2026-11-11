#include "armfw/calstore.h"

#include <string.h>

/* ---------------- CRC-32 (nibble table: 64 bytes of rodata) ---------------- */
static const uint32_t k_crc32_nib[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u,
    0x4DB26158u, 0x5005713Cu, 0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

uint32_t calstore_crc32(const uint8_t *d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        c = (c >> 4) ^ k_crc32_nib[c & 0xFu];
        c = (c >> 4) ^ k_crc32_nib[c & 0xFu];
    }
    return c ^ 0xFFFFFFFFu;
}

/* ---------------- record ---------------- */
void calstore_encode(const armfw_config_t *cfg, uint32_t seq, uint8_t out[CALSTORE_SLOT_LEN]) {
    memset(out, 0, CALSTORE_SLOT_LEN);
    armfw_put_u32(&out[0], CALSTORE_MAGIC);
    armfw_put_u16(&out[4], CALSTORE_VERSION);
    armfw_put_u16(&out[6], CALSTORE_PAYLOAD_LEN);
    armfw_put_u32(&out[8], seq);
    uint8_t *p = &out[12];
    for (int j = 0; j < ARMFW_N_JOINTS; j++, p += 12) {
        const joint_cfg_t *jc = &cfg->joint[j];
        armfw_put_i16(&p[0], (int16_t)jc->offset_ticks);
        p[2] = (uint8_t)jc->direction;
        p[3] = 0;
        armfw_put_i16(&p[4], jc->limit_min_mrad);
        armfw_put_i16(&p[6], jc->limit_max_mrad);
        armfw_put_i16(&p[8], jc->vmax_mrad_s);
        armfw_put_i16(&p[10], jc->current_limit_ma);
    }
    armfw_put_u16(&p[0], cfg->wd_hold_ms);
    armfw_put_u16(&p[2], cfg->wd_off_ms);
    armfw_put_u32(&out[CALSTORE_CRC_OFF], calstore_crc32(out, CALSTORE_CRC_OFF));
}

bool calstore_decode(const uint8_t rec[CALSTORE_SLOT_LEN], armfw_config_t *cfg, uint32_t *seq) {
    if (armfw_get_u32(&rec[0]) != CALSTORE_MAGIC) return false;
    if (armfw_get_u16(&rec[4]) != CALSTORE_VERSION) return false;
    if (armfw_get_u16(&rec[6]) != CALSTORE_PAYLOAD_LEN) return false;
    if (armfw_get_u32(&rec[CALSTORE_CRC_OFF]) != calstore_crc32(rec, CALSTORE_CRC_OFF)) return false;
    armfw_config_t c;
    config_defaults(&c); /* servo IDs and anything not stored */
    const uint8_t *p = &rec[12];
    for (int j = 0; j < ARMFW_N_JOINTS; j++, p += 12) {
        joint_cfg_t *jc = &c.joint[j];
        if (p[3] != 0) return false;
        jc->offset_ticks = armfw_get_i16(&p[0]);
        jc->direction = (int8_t)p[2];
        jc->limit_min_mrad = armfw_get_i16(&p[4]);
        jc->limit_max_mrad = armfw_get_i16(&p[6]);
        jc->vmax_mrad_s = armfw_get_i16(&p[8]);
        jc->current_limit_ma = armfw_get_i16(&p[10]);
    }
    c.wd_hold_ms = armfw_get_u16(&p[0]);
    c.wd_off_ms = armfw_get_u16(&p[2]);
    if (!config_validate(&c)) return false;
    *cfg = c;
    *seq = armfw_get_u32(&rec[8]);
    return true;
}

/* ---------------- flash helpers ---------------- */
static bool read_slot(const calstore_t *cs, int page, uint32_t slot, uint8_t *buf) {
    return cs->fp->read(cs->fp->ctx, (uint32_t)page, slot * CALSTORE_SLOT_LEN, buf, CALSTORE_SLOT_LEN);
}

static bool is_blank(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++)
        if (b[i] != 0xFFu) return false;
    return true;
}

/* blank = readable and all 0xFF */
static bool slot_blank(const calstore_t *cs, int page, uint32_t slot) {
    uint8_t b[CALSTORE_SLOT_LEN];
    return read_slot(cs, page, slot, b) && is_blank(b, sizeof b);
}

/* ---------------- load ---------------- */
bool calstore_init(calstore_t *cs, const flash_port_t *fp, armfw_config_t *cfg) {
    memset(cs, 0, sizeof(*cs));
    cs->fp = fp;
    cs->active_page = -1;
    cs->target_page = -1;
    config_defaults(cfg);
    if (!fp || fp->page_size < CALSTORE_SLOT_LEN) {
        cs->fp = NULL;
        return false;
    }
    cs->slots_per_page = fp->page_size / CALSTORE_SLOT_LEN;

    bool found = false;
    uint32_t best_seq = 0;
    int best_page = -1;
    uint32_t last_used[2] = {0, 0}; /* 1 + index of the last non-blank slot */
    for (int page = 0; page < 2; page++) {
        for (uint32_t s = 0; s < cs->slots_per_page; s++) {
            uint8_t b[CALSTORE_SLOT_LEN];
            bool ok = read_slot(cs, page, s, b);
            if (ok && is_blank(b, sizeof b)) continue;
            last_used[page] = s + 1;
            armfw_config_t c;
            uint32_t seq;
            if (ok && calstore_decode(b, &c, &seq)) {
                cs->valid_records++;
                if (seq > cs->seq) cs->seq = seq;
                if (!found || seq > best_seq) {
                    found = true;
                    best_seq = seq;
                    best_page = page;
                    *cfg = c;
                }
            } else {
                cs->garbage_slots++;
            }
        }
    }
    if (found) {
        cs->active_page = best_page;
        cs->next_slot = last_used[best_page];
        cs->loaded_seq = best_seq;
    }
    return found;
}

/* ---------------- save state machine ---------------- */
static int fail(calstore_t *cs) {
    cs->state = CS_IDLE;
    cs->saves_failed++;
    return -1;
}

/* Start an erase of page p and wait for it. */
static int begin_erase(calstore_t *cs, int p) {
    cs->target_page = p;
    cs->target_slot = 0;
    cs->polls = 0;
    if (!cs->fp->erase_start(cs->fp->ctx, (uint32_t)p)) return fail(cs);
    cs->state = CS_ERASE_WAIT;
    return 0;
}

/* Pick the next writable slot at or after (page, slot); erase the other page when the current one
 * is exhausted. Never touches the page holding the newest valid record except to append. */
static int choose_target(calstore_t *cs, int page, uint32_t slot) {
    if (page < 0) return begin_erase(cs, 0);
    for (; slot < cs->slots_per_page; slot++) {
        if (slot_blank(cs, page, slot)) {
            cs->target_page = page;
            cs->target_slot = slot;
            cs->dw = 0;
            cs->state = CS_PROGRAM;
            return 0;
        }
        cs->burned_slots++; /* not blank: skip it (torn earlier write) */
    }
    /* page exhausted: switch, unless that would erase the newest record */
    int other = 1 - page;
    if (other == cs->active_page) return fail(cs); /* cannot happen: page != active only after erase */
    return begin_erase(cs, other);
}

static int retry_next_slot(calstore_t *cs) {
    cs->burned_slots++;
    if (++cs->retries > CALSTORE_MAX_RETRIES) return fail(cs);
    return choose_target(cs, cs->target_page, cs->target_slot + 1);
}

int calstore_save_start(calstore_t *cs, const armfw_config_t *cfg) {
    if (!cs->fp) return -2;
    if (cs->state != CS_IDLE) return -1;
    cs->seq++;
    calstore_encode(cfg, cs->seq, cs->rec);
    cs->retries = 0;
    if (choose_target(cs, cs->active_page, cs->next_slot) < 0) cs->fail_pending = true;
    return 0;
}

static int start_program(calstore_t *cs) {
    uint32_t off = cs->target_slot * CALSTORE_SLOT_LEN + cs->dw * 8u;
    cs->polls = 0;
    if (!cs->fp->program_start(cs->fp->ctx, (uint32_t)cs->target_page, off, &cs->rec[cs->dw * 8u]))
        return retry_next_slot(cs);
    cs->state = CS_PROGRAM_WAIT;
    return 0;
}

int calstore_service(calstore_t *cs) {
    if (!cs->fp) return 0;
    if (cs->fail_pending) { /* a save that failed while being started is reported here once */
        cs->fail_pending = false;
        return -1;
    }
    switch (cs->state) {
    case CS_ERASE_WAIT: {
        int r = cs->fp->poll(cs->fp->ctx);
        if (r == 0) {
            if (++cs->polls > CALSTORE_MAX_POLLS) return fail(cs);
            return 0;
        }
        if (r < 0) return fail(cs);
        cs->erases[cs->target_page]++;
        /* the newest record (if any) lives in the other page: this page is now the append target */
        if (!slot_blank(cs, cs->target_page, 0)) return fail(cs); /* erase did not take */
        cs->target_slot = 0;
        cs->dw = 0;
        cs->state = CS_PROGRAM;
        return start_program(cs);
    }
    case CS_PROGRAM:
        return start_program(cs);
    case CS_PROGRAM_WAIT: {
        int r = cs->fp->poll(cs->fp->ctx);
        if (r == 0) {
            if (++cs->polls > CALSTORE_MAX_POLLS) return fail(cs);
            return 0;
        }
        if (r < 0) return retry_next_slot(cs);
        if (++cs->dw < CALSTORE_SLOT_LEN / 8u) return start_program(cs);
        cs->state = CS_VERIFY;
        return 0;
    }
    case CS_VERIFY: {
        uint8_t b[CALSTORE_SLOT_LEN];
        if (!read_slot(cs, cs->target_page, cs->target_slot, b) || memcmp(b, cs->rec, sizeof b) != 0)
            return retry_next_slot(cs);
        cs->active_page = cs->target_page;
        cs->next_slot = cs->target_slot + 1;
        cs->state = CS_IDLE;
        cs->saves_ok++;
        return 1;
    }
    case CS_IDLE:
    default:
        return 0;
    }
}
