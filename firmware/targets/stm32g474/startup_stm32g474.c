/* Startup for STM32G474 in C: vector table, reset handler (.data copy, .bss zero, FPU enable),
 * weak default handlers. Linker symbols come from stm32g474re.ld. */
#include <stdint.h>

#include "stm32g4_min.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);
void SystemInit(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_ALIAS;
void HardFault_Handler(void) WEAK_ALIAS;
void MemManage_Handler(void) WEAK_ALIAS;
void BusFault_Handler(void) WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;
void DMA1_Channel1_IRQHandler(void) WEAK_ALIAS;
void DMA1_Channel2_IRQHandler(void) WEAK_ALIAS;
void USART1_IRQHandler(void) WEAK_ALIAS;
void EXTI15_10_IRQHandler(void) WEAK_ALIAS;
void TIM6_DAC_IRQHandler(void) WEAK_ALIAS;
void LPUART1_IRQHandler(void) WEAK_ALIAS;
void USB_HP_IRQHandler(void) WEAK_ALIAS;
void USB_LP_IRQHandler(void) WEAK_ALIAS;
void USBWakeUp_IRQHandler(void) WEAK_ALIAS;

#define N_IRQ 102 /* STM32G474: IRQ0..IRQ101 - verify against RM0440 */
#define USB_HP_IRQn 19 /* checked against ST's stm32g474xx.h (tools/check_regs.py) */
#define USB_LP_IRQn 20
#define USBWakeUp_IRQn 42

typedef void (*vector_t)(void);

/* 16 core vectors + device IRQs. Unlisted IRQs go to Default_Handler: the range initializer fills
 * every IRQ slot, then the used ones are overridden (intentional, hence the pragma). */
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used)) const vector_t g_vectors[16 + N_IRQ] = {
    [0] = (vector_t)(uintptr_t)&_estack,
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 ... 16 + N_IRQ - 1] = Default_Handler,
    [16 + DMA1_Channel1_IRQn] = DMA1_Channel1_IRQHandler,
    [16 + DMA1_Channel2_IRQn] = DMA1_Channel2_IRQHandler,
    [16 + USB_HP_IRQn] = USB_HP_IRQHandler,
    [16 + USB_LP_IRQn] = USB_LP_IRQHandler,
    [16 + USBWakeUp_IRQn] = USBWakeUp_IRQHandler,
    [16 + USART1_IRQn] = USART1_IRQHandler,
    [16 + EXTI15_10_IRQn] = EXTI15_10_IRQHandler,
    [16 + TIM6_DAC_IRQn] = TIM6_DAC_IRQHandler,
    [16 + LPUART1_IRQn] = LPUART1_IRQHandler,
};

void Reset_Handler(void) {
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    SCB_CPACR |= (0xFu << 20); /* CP10/CP11 full access: enable the FPU before any float code */
    __asm volatile("dsb\n isb");
    SystemInit();
    main();
    for (;;) {
    }
}

void Default_Handler(void) {
    /* Unexpected interrupt or fault: servo power is cut by the hardware e-stop loop / watchdog
     * timeout on the host side; here we just stop. A debugger shows the active vector in IPSR. */
    for (;;) {
    }
}
