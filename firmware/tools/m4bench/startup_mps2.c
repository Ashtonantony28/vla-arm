/* Minimal startup for QEMU mps2-an386 (code at 0x0, RAM at 0x20000000). */
#include <stdint.h>
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void Reset_Handler(void) {
    uint32_t *s = &_sidata, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss;) *d++ = 0;
    *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;
    __asm volatile("dsb\n isb");
    main();
    for (;;) {
    }
}
static void hang(void) { for (;;) { } }
__attribute__((section(".isr_vector"), used)) void (*const vectors[16])(void) = {
    (void (*)(void))(uintptr_t)&_estack, Reset_Handler, hang, hang, hang, hang, hang, 0, 0, 0, 0, hang, hang, 0, hang, hang};
