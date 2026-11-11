/* Host-link byte rings between the 1 kHz control ISR and a packet transport (USB CDC) serviced
 * from the background loop. Portable C11; the transport is reached through host_link_io_t, so the
 * glue is unit tested on the host with a fake CDC.
 *
 *   control ISR (TIM6)                     background loop (tud_task + host_link_pump)
 *   host_read()  <-- rx ring <-- pump <-- tud_cdc_read()      (RX: transport -> ring)
 *   host_write() --> tx ring --> pump --> tud_cdc_write()     (TX: ring -> transport)
 *
 * Each ring has exactly one producer and one consumer (single-core Cortex-M4: ISR vs. thread), so
 * a free-running head/tail pair with acquire/release ordering is enough; no locks, no IRQ masking.
 */
#ifndef ARMFW_HOST_LINK_H
#define ARMFW_HOST_LINK_H

#include <stdatomic.h>

#include "armfw/common.h"

typedef struct {
    uint8_t *buf;
    uint32_t mask;          /* size - 1, size a power of two */
    atomic_uint_fast32_t head; /* written by the producer only */
    atomic_uint_fast32_t tail; /* written by the consumer only */
} byte_ring_t;

void ring_init(byte_ring_t *r, uint8_t *storage, uint32_t size_pow2);
uint32_t ring_used(const byte_ring_t *r);
uint32_t ring_free(const byte_ring_t *r);
/* producer side */
uint32_t ring_write(byte_ring_t *r, const uint8_t *d, uint32_t n); /* as much as fits */
/* consumer side */
uint32_t ring_read(byte_ring_t *r, uint8_t *d, uint32_t cap);
/* zero-copy consumer access: pointer to the longest contiguous readable run */
uint32_t ring_peek(const byte_ring_t *r, const uint8_t **p);
void ring_consume(byte_ring_t *r, uint32_t n);
/* consumer side: drop everything currently queued */
void ring_discard(byte_ring_t *r);

/* Transport operations (TinyUSB CDC on the target, a fake in tests). */
typedef struct {
    void *ctx;
    bool (*connected)(void *ctx);                 /* host has the port open (DTR) */
    uint32_t (*rx_available)(void *ctx);
    uint32_t (*read)(void *ctx, uint8_t *buf, uint32_t n);
    uint32_t (*write_available)(void *ctx);
    uint32_t (*write)(void *ctx, const uint8_t *buf, uint32_t n);
    void (*flush)(void *ctx);
} host_link_io_t;

typedef struct {
    byte_ring_t rx, tx;
    uint32_t tx_rejected_frames; /* host_write calls refused for lack of space */
    uint32_t tx_discarded_bytes; /* dropped because no host had the port open */
    uint32_t rx_bytes, tx_bytes;
} host_link_t;

void host_link_init(host_link_t *l, uint8_t *rx_storage, uint32_t rx_size, uint8_t *tx_storage,
                    uint32_t tx_size);
/* ISR side: hal_port.host_read / host_write. host_write takes all n bytes (one whole frame) or
 * none, so a full ring never emits a truncated frame. */
size_t host_link_read(host_link_t *l, uint8_t *buf, size_t cap);
size_t host_link_write(host_link_t *l, const uint8_t *buf, size_t n);
/* Background side, after tud_task(): move received bytes into the rx ring (only as many as fit:
 * the rest stays in the transport, which NAKs the host = flow control) and queued tx bytes into
 * the transport, then flush. With no host connected, queued tx bytes are discarded so a newly
 * opened port does not start with stale frames. */
void host_link_pump(host_link_t *l, const host_link_io_t *io);

#endif
