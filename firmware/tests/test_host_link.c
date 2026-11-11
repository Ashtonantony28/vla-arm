/* Host-link glue (USB CDC <-> control ISR rings) against a fake CDC endpoint: byte-exact transport
 * of random frame streams under random packetisation and back-pressure, whole-frame writes,
 * disconnect handling, and a real threaded SPSC run (producer and consumer on two threads, the
 * same pattern as the TIM6 ISR vs. the background loop). */
#include <pthread.h>

#include "armfw/host_link.h"
#include "armfw/proto.h"
#include "tt.h"

/* ---- fake CDC: an RX FIFO filled by the "host", a TX sink with limited room per call ---- */
typedef struct {
    uint8_t rx[1 << 16];
    uint32_t rx_head, rx_tail;
    uint8_t tx[1 << 20];
    uint32_t tx_len;
    uint32_t tx_room;   /* bytes the endpoint FIFO accepts until the next "USB frame" */
    bool connected;
    uint32_t flushes;
    tt_rng_t rng;
} fake_cdc_t;

static bool f_connected(void *c) { return ((fake_cdc_t *)c)->connected; }
static uint32_t f_rx_avail(void *c) {
    fake_cdc_t *f = c;
    return f->rx_head - f->rx_tail;
}
static uint32_t f_read(void *c, uint8_t *b, uint32_t n) {
    fake_cdc_t *f = c;
    uint32_t k = 0;
    /* TinyUSB may return fewer bytes than asked */
    uint32_t lim = 1 + tt_rand(&f->rng) % 64;
    while (k < n && k < lim && f->rx_tail != f->rx_head) b[k++] = f->rx[f->rx_tail++ % sizeof f->rx];
    return k;
}
static uint32_t f_wavail(void *c) { return ((fake_cdc_t *)c)->tx_room; }
static uint32_t f_write(void *c, const uint8_t *b, uint32_t n) {
    fake_cdc_t *f = c;
    if (n > f->tx_room) n = f->tx_room;
    memcpy(&f->tx[f->tx_len], b, n);
    f->tx_len += n;
    f->tx_room -= n;
    return n;
}
static void f_flush(void *c) { ((fake_cdc_t *)c)->flushes++; }

static fake_cdc_t g_cdc;
static host_link_io_t g_io = {&g_cdc, f_connected, f_rx_avail, f_read, f_wavail, f_write, f_flush};
static uint8_t g_rxs[1024], g_txs[2048];

static void cdc_host_send(fake_cdc_t *f, const uint8_t *d, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) f->rx[f->rx_head++ % sizeof f->rx] = d[i];
}

TT_TEST(ring_basics_and_wrap) {
    uint8_t st[16];
    byte_ring_t r;
    ring_init(&r, st, 16);
    TT_EQ(ring_free(&r), 16);
    uint8_t in[40], out[40];
    for (int i = 0; i < 40; i++) in[i] = (uint8_t)i;
    uint32_t pos_in = 0, pos_out = 0;
    tt_rng_t rng = {5};
    /* random write/read sizes across many wraps of the free-running indices */
    for (int k = 0; k < 20000; k++) {
        uint8_t chunk[16];
        uint32_t n = (uint32_t)tt_randi(&rng, 0, 16);
        for (uint32_t i = 0; i < n; i++) chunk[i] = (uint8_t)(pos_in + i);
        uint32_t w = ring_write(&r, chunk, n);
        TT_ASSERT(w <= n);
        pos_in += w;
        uint32_t m = (uint32_t)tt_randi(&rng, 0, 16);
        uint32_t got = ring_read(&r, out, m);
        for (uint32_t i = 0; i < got; i++) TT_EQ(out[i], (uint8_t)(pos_out + i));
        pos_out += got;
        TT_EQ(ring_used(&r), pos_in - pos_out);
    }
    /* free-running counters near 2^32 */
    atomic_store(&r.head, 0xFFFFFFF0u);
    atomic_store(&r.tail, 0xFFFFFFF0u);
    TT_EQ(ring_write(&r, in, 16), 16);
    TT_EQ(ring_write(&r, in, 1), 0);
    TT_EQ(ring_read(&r, out, 40), 16);
    TT_MEM_EQ(out, in, 16);
    ring_write(&r, in, 5);
    ring_discard(&r);
    TT_EQ(ring_used(&r), 0);
}

TT_TEST(whole_frames_only_on_tx) {
    host_link_t l;
    host_link_init(&l, g_rxs, sizeof g_rxs, g_txs, sizeof g_txs);
    uint8_t frame[74];
    memset(frame, 0xAB, sizeof frame);
    int accepted = 0;
    for (int i = 0; i < 100; i++) accepted += host_link_write(&l, frame, sizeof frame) == sizeof frame;
    TT_EQ(accepted, (int)(sizeof g_txs / sizeof frame)); /* 27 frames fit in 2 KB */
    TT_EQ(ring_used(&l.tx), (uint32_t)accepted * sizeof frame);
    TT_EQ(l.tx_rejected_frames, (uint32_t)(100 - accepted));
}

TT_TEST(stream_integrity_random_packetisation) {
    /* the board emits real protocol frames at 1 kHz-ish rates while the "USB" side drains 0..64
     * bytes per pump; the host side must reassemble exactly the accepted frames, in order. */
    host_link_t l;
    host_link_init(&l, g_rxs, sizeof g_rxs, g_txs, sizeof g_txs);
    memset(&g_cdc, 0, sizeof g_cdc);
    g_cdc.connected = true;
    g_cdc.rng.s = 99;
    tt_rng_t rng = {1234};
    uint8_t seq = 0;
    uint32_t accepted = 0;
    static uint8_t expect[1 << 20];
    uint32_t expect_len = 0;
    for (int step = 0; step < 20000; step++) {
        /* ISR: 0-3 frames of random type/length */
        int nf = tt_randi(&rng, 0, 3);
        for (int k = 0; k < nf; k++) {
            uint8_t pl[PROTO_MAX_PAYLOAD], fr[PROTO_MAX_FRAME];
            size_t len = (size_t)tt_randi(&rng, 0, 80);
            for (size_t i = 0; i < len; i++) pl[i] = (uint8_t)tt_rand(&rng);
            size_t n = proto_encode(fr, sizeof fr, PROTO_T_STATE, seq, pl, len);
            if (host_link_write(&l, fr, n) == n) {
                memcpy(&expect[expect_len], fr, n);
                expect_len += (uint32_t)n;
                accepted++;
                seq++;
            }
        }
        /* background loop: USB FIFO room for this "frame" */
        g_cdc.tx_room = (uint32_t)tt_randi(&rng, 0, 64);
        host_link_pump(&l, &g_io);
        if (expect_len > sizeof expect - 4096) break;
    }
    /* drain */
    for (int i = 0; i < 1000 && ring_used(&l.tx); i++) {
        g_cdc.tx_room = 64;
        host_link_pump(&l, &g_io);
    }
    TT_EQ(g_cdc.tx_len, expect_len);
    TT_MEM_EQ(g_cdc.tx, expect, expect_len);
    /* and the host-side parser sees every accepted frame, CRC-valid, in sequence */
    proto_parser_t p;
    proto_parser_init(&p);
    uint32_t frames = 0;
    proto_frame_t f;
    for (uint32_t i = 0; i < g_cdc.tx_len; i++) {
        proto_parser_feed(&p, g_cdc.tx[i]);
        while (proto_parser_poll(&p, &f)) {
            TT_EQ(f.seq, (uint8_t)frames);
            frames++;
        }
    }
    TT_EQ(frames, accepted);
    TT_ASSERT(l.tx_rejected_frames > 0); /* back-pressure was exercised */
}

TT_TEST(rx_backpressure_never_drops) {
    host_link_t l;
    host_link_init(&l, g_rxs, sizeof g_rxs, g_txs, sizeof g_txs);
    memset(&g_cdc, 0, sizeof g_cdc);
    g_cdc.connected = true;
    g_cdc.rng.s = 7;
    tt_rng_t rng = {77};
    static uint8_t sent[40000], got[40000];
    for (uint32_t i = 0; i < sizeof sent; i++) sent[i] = (uint8_t)tt_rand(&rng);
    cdc_host_send(&g_cdc, sent, sizeof sent); /* a burst much larger than the 1 KB ring */
    uint32_t n = 0;
    for (int step = 0; step < 100000 && n < sizeof got; step++) {
        host_link_pump(&l, &g_io);
        TT_ASSERT(ring_used(&l.rx) <= sizeof g_rxs);
        /* control step reads at most CONTROL_MAX_RX_PER_STEP-ish, often less */
        n += (uint32_t)host_link_read(&l, &got[n], (size_t)tt_randi(&rng, 0, 300));
    }
    TT_EQ(n, sizeof sent);
    TT_MEM_EQ(got, sent, sizeof sent);
}

TT_TEST(disconnected_host_discards_stale_tx) {
    host_link_t l;
    host_link_init(&l, g_rxs, sizeof g_rxs, g_txs, sizeof g_txs);
    memset(&g_cdc, 0, sizeof g_cdc);
    g_cdc.connected = false;
    uint8_t fr[74] = {0xA5, 0x5A};
    for (int i = 0; i < 10; i++) host_link_write(&l, fr, sizeof fr);
    g_cdc.tx_room = 64;
    host_link_pump(&l, &g_io);
    TT_EQ(g_cdc.tx_len, 0);
    TT_EQ(ring_used(&l.tx), 0);
    TT_EQ(l.tx_discarded_bytes, 740);
    TT_EQ(g_cdc.flushes, 0);
    g_cdc.connected = true;
    host_link_write(&l, fr, sizeof fr);
    for (int i = 0; i < 3; i++) {
        g_cdc.tx_room = 64;
        host_link_pump(&l, &g_io);
    }
    TT_EQ(g_cdc.tx_len, 74);
    TT_ASSERT(g_cdc.flushes >= 2);
}

/* ---- two real threads: producer (ISR role) and consumer (background role) ---- */
#define SPSC_BYTES 4000000u
static byte_ring_t g_r;
static uint8_t g_rst[256];

static void *producer(void *arg) {
    (void)arg;
    uint32_t pos = 0;
    tt_rng_t rng = {3};
    while (pos < SPSC_BYTES) {
        uint8_t chunk[64];
        uint32_t n = 1 + tt_rand(&rng) % 64;
        if (n > SPSC_BYTES - pos) n = SPSC_BYTES - pos;
        for (uint32_t i = 0; i < n; i++) chunk[i] = (uint8_t)((pos + i) * 2654435761u >> 24);
        uint32_t w = 0;
        while (w < n) w += ring_write(&g_r, chunk + w, n - w);
        pos += n;
    }
    return NULL;
}

TT_TEST(spsc_two_threads) {
    ring_init(&g_r, g_rst, sizeof g_rst);
    pthread_t th;
    TT_EQ(pthread_create(&th, NULL, producer, NULL), 0);
    uint32_t pos = 0, bad = 0;
    while (pos < SPSC_BYTES) {
        const uint8_t *p;
        uint32_t run = ring_peek(&g_r, &p);
        for (uint32_t i = 0; i < run; i++)
            if (p[i] != (uint8_t)((pos + i) * 2654435761u >> 24)) bad++;
        ring_consume(&g_r, run);
        pos += run;
    }
    pthread_join(th, NULL);
    TT_EQ(bad, 0);
    TT_EQ(ring_used(&g_r), 0);
}

TT_MAIN()
