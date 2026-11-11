/* Hardware abstraction used by the portable core. The core never touches registers; a target
 * (targets/stm32g474/) or the host simulation (sim/) supplies these functions.
 *
 * Contract (all calls are made from the 1 kHz control step, never concurrently):
 *  - time_us:        free-running microsecond counter, wraps at 2^32.
 *  - host_read:      non-blocking; copy up to cap received host-link bytes, return the count.
 *  - host_write:     non-blocking; queue n bytes to the host link, return the count accepted.
 *  - servo_txrx:     start one half-duplex bus transaction: transmit n bytes (DMA), then switch the
 *                    line to receive. Must not block for the duration of the transfer. The tx
 *                    buffer is only valid during the call, so the HAL copies it.
 *  - servo_read:     non-blocking; bytes received on the servo bus since the last call (replies to
 *                    the previous cycle's transaction). Echoes of our own TX are tolerated.
 *  - estop_loop_closed: hardware e-stop loop state, true = closed (OK). Should latch an opening
 *                    seen by the EXTI ISR until read.
 *  - adc_read:       non-blocking; copy the latest raw ADC codes (see armfw/analog.h) and return
 *                    true if a conversion completed since the previous call. Scaling, filtering
 *                    and thresholds are done by the core.
 *  - flash:          optional (NULL = calibration is RAM only). Two erase pages reserved for the
 *                    calibration store; see armfw/calstore.h.
 */
#ifndef ARMFW_HAL_PORT_H
#define ARMFW_HAL_PORT_H

#include "armfw/analog.h"
#include "armfw/common.h"

/* Calibration flash: exactly two pages (0 = A, 1 = B) of page_size bytes each. Programming is in
 * 8-byte double words, each double word at most once per erase (STM32G4 ECC flash rules).
 * erase_start / program_start start an operation and return false if it could not be started;
 * poll returns 0 while busy, 1 when the last operation finished OK, -1 on a flash error.
 * read returns false if the data could not be read reliably (e.g. an ECC double error on a
 * double word torn by a power cut); the caller treats such a region as garbage. */
typedef struct flash_port {
    void *ctx;
    uint32_t page_size;
    bool (*read)(void *ctx, uint32_t page, uint32_t off, uint8_t *buf, uint32_t n);
    bool (*erase_start)(void *ctx, uint32_t page);
    bool (*program_start)(void *ctx, uint32_t page, uint32_t off, const uint8_t dw[8]);
    int (*poll)(void *ctx);
} flash_port_t;

typedef struct hal_port {
    void *ctx;
    uint32_t (*time_us)(void *ctx);
    size_t (*host_read)(void *ctx, uint8_t *buf, size_t cap);
    size_t (*host_write)(void *ctx, const uint8_t *buf, size_t n);
    void (*servo_txrx)(void *ctx, const uint8_t *tx, size_t n);
    size_t (*servo_read)(void *ctx, uint8_t *buf, size_t cap);
    bool (*estop_loop_closed)(void *ctx);
    bool (*adc_read)(void *ctx, adc_raw_t *out);
    const flash_port_t *flash;
} hal_port_t;

#endif
