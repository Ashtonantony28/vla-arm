/* USB FS CDC glue - STUB.
 *
 * The production host link is USB CDC (spec/protocol.md "Transport"). The plan is TinyUSB
 * (MIT, supports the STM32G4 USB FS device peripheral). Integration sketch:
 *
 *   - clocks: HSI48 + CRS synced to USB SOF (RCC->CRRCR HSI48ON is set in SystemInit), USB on PA11/PA12
 *   - call tud_init(0) from usb_cdc_glue_init(), tud_task() from the main loop (not the 1 kHz ISR),
 *     USB_LP_IRQHandler -> tud_int_handler(0)
 *   - tud_cdc_rx_cb(): tud_cdc_read() into an SPSC ring that replaces the LPUART RX ring
 *   - host_write(): copy into an SPSC TX ring; the main loop moves it with tud_cdc_write() +
 *     tud_cdc_write_flush(). At 200 Hz STATE (66 + 8 bytes) plus ACKs this is < 20 kB/s.
 *   - the hal_port host_read/host_write function pointers are switched from LPUART to these
 *     rings when ARMFW_HOST_LINK_USB is defined.
 *
 * Until then the LPUART link (ST-LINK VCP on the NUCLEO) is used for bring-up.
 */
#include <stddef.h>
#include <stdint.h>

void usb_cdc_glue_init(void) {
    /* TODO: tud_init(0) once TinyUSB is vendored. */
}

void USB_LP_IRQHandler(void) {
    /* TODO: tud_int_handler(0) */
}
