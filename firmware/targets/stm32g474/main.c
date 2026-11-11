/* Bare-metal main: init, then the TIM6 1 kHz interrupt runs control_step(). The background loop
 * runs the USB stack (TinyUSB tud_task + host-link ring pump) and sleeps between interrupts. Step execution time is measured with the DWT cycle counter for bring-up. */
#include "armfw/control.h"
#include "board.h"
#include "stm32g4_min.h"

void hal_stm32_init(void);
void hal_stm32_start_tick(void);
void hal_stm32_flush(void);
void hal_stm32_init_flash(void);
void usb_cdc_glue_init(void);
void usb_cdc_glue_task(void);
extern hal_port_t g_hal_stm32;

static control_t s_ctl;
volatile uint32_t g_step_cycles_max; /* inspect with a debugger during bring-up */
volatile uint32_t g_step_cycles_last;
volatile uint32_t g_ms_ticks; /* TinyUSB time base */

void TIM6_DAC_IRQHandler(void) {
    TIM6->SR = 0;
    g_ms_ticks++;
    uint32_t c0 = DWT_CYCCNT;
    control_step(&s_ctl);
    hal_stm32_flush();
    uint32_t c = DWT_CYCCNT - c0;
    g_step_cycles_last = c;
    if (c > g_step_cycles_max) g_step_cycles_max = c;
    if (s_ctl.steps % 500u == 0u) GPIOA->ODR ^= 1u << 5; /* 1 Hz heartbeat */
}

int main(void) {
    hal_stm32_init();
    hal_stm32_init_flash();
    control_init(&s_ctl, &g_hal_stm32); /* loads the calibration from flash (defaults if none) */
#ifdef ARMFW_HOST_LINK_USB
    usb_cdc_glue_init();
#endif
    hal_stm32_start_tick();
    for (;;) {
#ifdef ARMFW_HOST_LINK_USB
        usb_cdc_glue_task(); /* USB stack + ring pump, outside the 1 kHz ISR */
#endif
        __asm volatile("wfi"); /* woken by the tick, USB or DMA interrupts */
    }
}
