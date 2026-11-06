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
 *  - bus_mv / bus_ma: latest servo-rail voltage (mV) and current (mA) from the board ADC.
 */
#ifndef ARMFW_HAL_PORT_H
#define ARMFW_HAL_PORT_H

#include "armfw/common.h"

typedef struct hal_port {
    void *ctx;
    uint32_t (*time_us)(void *ctx);
    size_t (*host_read)(void *ctx, uint8_t *buf, size_t cap);
    size_t (*host_write)(void *ctx, const uint8_t *buf, size_t n);
    void (*servo_txrx)(void *ctx, const uint8_t *tx, size_t n);
    size_t (*servo_read)(void *ctx, uint8_t *buf, size_t cap);
    bool (*estop_loop_closed)(void *ctx);
    uint16_t (*bus_mv)(void *ctx);
    int16_t (*bus_ma)(void *ctx);
} hal_port_t;

#endif
