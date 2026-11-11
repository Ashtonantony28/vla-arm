#include "fake_flash.h"

#include <string.h>

static uint32_t rnd(fake_flash_t *f) {
    f->rng = f->rng * 1664525u + 1013904223u;
    return f->rng >> 8;
}

void fake_flash_init(fake_flash_t *f) {
    memset(f, 0, sizeof(*f));
    memset(f->mem, 0xFF, sizeof f->mem);
    f->budget = -1;
    f->erase_tear_dw = -1;
    f->fail_program_nth = -1;
    f->corrupt_program_nth = -1;
    f->rng = 0xC0FFEEu;
    f->result = 1;
}

void fake_flash_power_on(fake_flash_t *f) {
    f->dead = false;
    f->budget = -1;
    f->busy_left = 0;
    f->result = 1;
    f->fail_program_nth = -1;
    f->corrupt_program_nth = -1;
}

static bool ff_read(void *ctx, uint32_t page, uint32_t off, uint8_t *buf, uint32_t n) {
    fake_flash_t *f = (fake_flash_t *)ctx;
    f->reads++;
    if (f->dead || page >= FF_PAGES || off > FF_PAGE_SIZE || n > FF_PAGE_SIZE - off) return false;
    for (uint32_t dw = off / 8u; dw * 8u < off + n; dw++)
        if (f->ecc_bad[page][dw]) return false;
    memcpy(buf, &f->mem[page][off], n);
    return true;
}

/* Spend `cost` budget units; returns how many were available (cost = complete). */
static int32_t spend(fake_flash_t *f, int32_t cost) {
    if (f->budget < 0) return cost;
    int32_t got = f->budget < cost ? f->budget : cost;
    f->budget -= got;
    if (got < cost) f->dead = true;
    return got;
}

static void tear_dw(fake_flash_t *f, uint32_t page, uint32_t dw, const uint8_t *want, int32_t good) {
    uint8_t *p = &f->mem[page][dw * 8u];
    switch (f->torn_mode) {
    case FF_TORN_BLANK:
        return; /* nothing visible */
    case FF_TORN_ECC:
        f->ecc_bad[page][dw] = 1;
        /* the contents are partially written as well */
        /* fall through */
    case FF_TORN_GARBAGE:
    default:
        for (int32_t i = 0; i < 8; i++) {
            uint8_t target = want ? want[i] : 0xFFu;
            if (i < good)
                p[i] = target;
            else
                p[i] = (uint8_t)(p[i] & (target | (uint8_t)rnd(f))); /* some bits made it to 0 */
        }
        return;
    }
}

static bool ff_erase_start(void *ctx, uint32_t page) {
    fake_flash_t *f = (fake_flash_t *)ctx;
    if (f->dead || page >= FF_PAGES) return false;
    int32_t got = spend(f, FF_ERASE_COST);
    if (got < FF_ERASE_COST) {
        /* torn erase: a prefix of double words is erased, one is in between, the rest is old */
        uint32_t n = f->erase_tear_dw >= 0 ? (uint32_t)f->erase_tear_dw : rnd(f) % FF_DW_PER_PAGE;
        if (n > FF_DW_PER_PAGE) n = FF_DW_PER_PAGE;
        memset(f->mem[page], 0xFF, n * 8u);
        memset(f->ecc_bad[page], 0, n);
        if (n < FF_DW_PER_PAGE) {
            uint8_t *p = &f->mem[page][n * 8u];
            for (int i = 0; i < 8; i++) p[i] = (uint8_t)(p[i] | (uint8_t)rnd(f)); /* half erased */
            if (f->torn_mode == FF_TORN_ECC) f->ecc_bad[page][n] = 1;
        }
        return true; /* the CPU never learns the outcome: power is gone */
    }
    memset(f->mem[page], 0xFF, FF_PAGE_SIZE);
    memset(f->ecc_bad[page], 0, FF_DW_PER_PAGE);
    f->erases[page]++;
    f->busy_left = f->busy_polls;
    f->result = 1;
    return true;
}

static bool ff_program_start(void *ctx, uint32_t page, uint32_t off, const uint8_t dw8[8]) {
    fake_flash_t *f = (fake_flash_t *)ctx;
    if (f->dead || page >= FF_PAGES || (off & 7u) || off + 8u > FF_PAGE_SIZE) return false;
    uint32_t dw = off / 8u;
    uint32_t nth = f->programs++;
    f->busy_left = f->busy_polls;
    uint8_t *p = &f->mem[page][off];
    bool erased = !f->ecc_bad[page][dw];
    for (int i = 0; i < 8; i++)
        if (p[i] != 0xFFu) erased = false;
    if (!erased || (int32_t)nth == f->fail_program_nth) {
        f->result = -1; /* PROGERR: target not erased (or injected error), nothing written */
        return true;
    }
    int32_t got = spend(f, 8);
    if (got < 8) {
        tear_dw(f, page, dw, dw8, got);
        return true;
    }
    memcpy(p, dw8, 8);
    if ((int32_t)nth == f->corrupt_program_nth) p[rnd(f) % 8u] ^= (uint8_t)(1u << (rnd(f) % 8u));
    f->result = 1;
    return true;
}

static int ff_poll(void *ctx) {
    fake_flash_t *f = (fake_flash_t *)ctx;
    if (f->dead) return -1;
    if (f->busy_left) {
        f->busy_left--;
        return 0;
    }
    return f->result;
}

void fake_flash_port(fake_flash_t *f, flash_port_t *port) {
    port->ctx = f;
    port->page_size = FF_PAGE_SIZE;
    port->read = ff_read;
    port->erase_start = ff_erase_start;
    port->program_start = ff_program_start;
    port->poll = ff_poll;
}
