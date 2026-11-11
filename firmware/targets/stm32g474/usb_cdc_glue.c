/* USB FS CDC host link (TinyUSB 0.21.0, STM32 FSDEV driver).
 *
 *   - clocks: HSI48 (enabled in SystemInit) is the USB kernel clock (RCC CCIPR.CLK48SEL reset value
 *     00 = HSI48), trimmed by the CRS from the host's 1 kHz SOF; USB peripheral clock on APB1.
 *   - pins PA11/PA12: the USB FS PHY takes them over when the peripheral is enabled (no AF
 *     setting, per ST's HAL); internal D+ pull-up via USB_BCDR.DPPU (done by TinyUSB's dcd_connect).
 *   - USB_HP / USB_LP / USBWakeUp IRQs -> tud_int_handler(0), priority 3 (below the 1 kHz control
 *     tick at 1, so the control step is never delayed by USB traffic).
 *   - tud_task() and host_link_pump() run in the background loop (usb_cdc_glue_task), never in
 *     the control ISR. The ISR only touches the host_link rings (host_link_read/write).
 *
 * Unverified (from memory, check on silicon): CRS SYNCSRC reset value = USB SOF (10b), and that
 * the PA11/PA12 MODER setting does not matter for the USB FS PHY on the G4.
 */
#include <stdbool.h>
#include <stdint.h>

#include "armfw/host_link.h"
#include "board.h"
#include "stm32g4xx.h"
#include "tusb.h"

static uint8_t s_rx_store[BOARD_USB_RX_RING];
static uint8_t s_tx_store[BOARD_USB_TX_RING];
static host_link_t s_link;

extern volatile uint32_t g_ms_ticks; /* main.c, incremented by the 1 kHz tick */

uint32_t tusb_time_millis_api(void) { return g_ms_ticks; }

static bool io_connected(void *ctx) {
    (void)ctx;
    return tud_cdc_connected();
}
static uint32_t io_rx_available(void *ctx) {
    (void)ctx;
    return tud_cdc_available();
}
static uint32_t io_read(void *ctx, uint8_t *buf, uint32_t n) {
    (void)ctx;
    return tud_cdc_read(buf, n);
}
static uint32_t io_write_available(void *ctx) {
    (void)ctx;
    return tud_cdc_write_available();
}
static uint32_t io_write(void *ctx, const uint8_t *buf, uint32_t n) {
    (void)ctx;
    return tud_cdc_write(buf, n);
}
static void io_flush(void *ctx) {
    (void)ctx;
    (void)tud_cdc_write_flush();
}

static const host_link_io_t s_io = {
    .ctx = 0,
    .connected = io_connected,
    .rx_available = io_rx_available,
    .read = io_read,
    .write_available = io_write_available,
    .write = io_write,
    .flush = io_flush,
};

void usb_cdc_glue_init(void) {
    host_link_init(&s_link, s_rx_store, sizeof s_rx_store, s_tx_store, sizeof s_tx_store);

    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;
    (void)RCC->APB1ENR1;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {
    }
    /* CRS: automatic HSI48 trimming on USB SOF (SYNCSRC reset value = USB SOF) */
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    NVIC_SetPriority(USB_HP_IRQn, 3);
    NVIC_SetPriority(USB_LP_IRQn, 3);
    NVIC_SetPriority(USBWakeUp_IRQn, 3);
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

/* Background loop: USB stack events, then move bytes between TinyUSB's FIFOs and the rings. */
void usb_cdc_glue_task(void) {
    tud_task();
    host_link_pump(&s_link, &s_io);
}

/* hal_port host_read / host_write (called from the control ISR). */
size_t usb_host_read(uint8_t *buf, size_t cap) { return host_link_read(&s_link, buf, cap); }
size_t usb_host_write(const uint8_t *buf, size_t n) { return host_link_write(&s_link, buf, n); }

void USB_HP_IRQHandler(void) { tud_int_handler(BOARD_TUD_RHPORT); }
void USB_LP_IRQHandler(void) { tud_int_handler(BOARD_TUD_RHPORT); }
void USBWakeUp_IRQHandler(void) { tud_int_handler(BOARD_TUD_RHPORT); }
