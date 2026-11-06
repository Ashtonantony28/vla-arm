/* Clock tree: HSI16 -> PLL (M=4, N=85, R=2) -> SYSCLK 170 MHz, range-1 boost mode, 4 flash wait
 * states, AHB/APB at 170 MHz. HSI48 on for USB. Sequence per RM0440 "Range 1 boost mode":
 * switch with AHB/2, wait >= 1 us, then AHB/1. */
#include "board.h"
#include "stm32g4_min.h"

void SystemInit(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR_CR5 &= ~PWR_CR5_R1MODE; /* range 1 boost (needed above 150 MHz) */
    FLASH_ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH_ACR & 0xFu) != FLASH_ACR_LATENCY_4WS) {
    }

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {
    }
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {
    }
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | ((4u - 1u) << RCC_PLLCFGR_PLLM_Pos) |
                   (85u << RCC_PLLCFGR_PLLN_Pos) | (0u << RCC_PLLCFGR_PLLR_Pos) /* R = 2 */ |
                   RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    RCC->CFGR = (RCC->CFGR & ~0xF3u) | RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {
    }
    for (volatile int i = 0; i < 200; i++) { /* >= 1 us at 85 MHz */
    }
    RCC->CFGR &= ~(0xFu << 4); /* AHB /1 */

    RCC->CRRCR |= RCC_CRRCR_HSI48ON; /* USB clock (with CRS trimming, not configured here) */
}
