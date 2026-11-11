#include "armfw/host_link.h"

#include <string.h>

void ring_init(byte_ring_t *r, uint8_t *storage, uint32_t size_pow2) {
    r->buf = storage;
    r->mask = size_pow2 - 1u;
    atomic_init(&r->head, 0u);
    atomic_init(&r->tail, 0u);
}

uint32_t ring_used(const byte_ring_t *r) {
    uint32_t h = (uint32_t)atomic_load_explicit(&((byte_ring_t *)r)->head, memory_order_acquire);
    uint32_t t = (uint32_t)atomic_load_explicit(&((byte_ring_t *)r)->tail, memory_order_acquire);
    return h - t;
}

uint32_t ring_free(const byte_ring_t *r) { return r->mask + 1u - ring_used(r); }

uint32_t ring_write(byte_ring_t *r, const uint8_t *d, uint32_t n) {
    uint32_t h = (uint32_t)atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t t = (uint32_t)atomic_load_explicit(&r->tail, memory_order_acquire);
    uint32_t space = r->mask + 1u - (h - t);
    if (n > space) n = space;
    uint32_t i = h & r->mask;
    uint32_t first = r->mask + 1u - i;
    if (first > n) first = n;
    memcpy(&r->buf[i], d, first);
    memcpy(r->buf, d + first, n - first);
    atomic_store_explicit(&r->head, h + n, memory_order_release);
    return n;
}

uint32_t ring_peek(const byte_ring_t *r, const uint8_t **p) {
    uint32_t t = (uint32_t)atomic_load_explicit(&((byte_ring_t *)r)->tail, memory_order_relaxed);
    uint32_t h = (uint32_t)atomic_load_explicit(&((byte_ring_t *)r)->head, memory_order_acquire);
    uint32_t used = h - t;
    uint32_t i = t & r->mask;
    uint32_t run = r->mask + 1u - i;
    *p = &r->buf[i];
    return used < run ? used : run;
}

void ring_consume(byte_ring_t *r, uint32_t n) {
    uint32_t t = (uint32_t)atomic_load_explicit(&r->tail, memory_order_relaxed);
    atomic_store_explicit(&r->tail, t + n, memory_order_release);
}

uint32_t ring_read(byte_ring_t *r, uint8_t *d, uint32_t cap) {
    uint32_t got = 0;
    while (got < cap) {
        const uint8_t *p;
        uint32_t run = ring_peek(r, &p);
        if (run == 0) break;
        if (run > cap - got) run = cap - got;
        memcpy(d + got, p, run);
        ring_consume(r, run);
        got += run;
    }
    return got;
}

void ring_discard(byte_ring_t *r) {
    uint32_t h = (uint32_t)atomic_load_explicit(&r->head, memory_order_acquire);
    atomic_store_explicit(&r->tail, h, memory_order_release);
}

void host_link_init(host_link_t *l, uint8_t *rx_storage, uint32_t rx_size, uint8_t *tx_storage,
                    uint32_t tx_size) {
    memset(l, 0, sizeof(*l));
    ring_init(&l->rx, rx_storage, rx_size);
    ring_init(&l->tx, tx_storage, tx_size);
}

size_t host_link_read(host_link_t *l, uint8_t *buf, size_t cap) {
    uint32_t c = cap > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)cap;
    return ring_read(&l->rx, buf, c);
}

size_t host_link_write(host_link_t *l, const uint8_t *buf, size_t n) {
    if (n > ring_free(&l->tx)) {
        l->tx_rejected_frames++;
        return 0;
    }
    return ring_write(&l->tx, buf, (uint32_t)n);
}

void host_link_pump(host_link_t *l, const host_link_io_t *io) {
    /* RX: transport -> ring, bounded by ring space (leftovers stay in the transport FIFO) */
    for (;;) {
        uint32_t avail = io->rx_available(io->ctx);
        uint32_t space = ring_free(&l->rx);
        if (avail == 0 || space == 0) break;
        uint8_t tmp[64];
        uint32_t n = avail < space ? avail : space;
        if (n > sizeof tmp) n = sizeof tmp;
        n = io->read(io->ctx, tmp, n);
        if (n == 0) break;
        ring_write(&l->rx, tmp, n);
        l->rx_bytes += n;
    }
    /* TX: ring -> transport */
    if (!io->connected(io->ctx)) {
        uint32_t u = ring_used(&l->tx);
        if (u) {
            ring_discard(&l->tx);
            l->tx_discarded_bytes += u;
        }
        return;
    }
    bool wrote = false;
    for (;;) {
        const uint8_t *p;
        uint32_t run = ring_peek(&l->tx, &p);
        uint32_t room = io->write_available(io->ctx);
        if (run == 0 || room == 0) break;
        if (run > room) run = room;
        uint32_t w = io->write(io->ctx, p, run);
        if (w == 0) break;
        ring_consume(&l->tx, w);
        l->tx_bytes += w;
        wrote = true;
    }
    if (wrote) io->flush(io->ctx);
}
