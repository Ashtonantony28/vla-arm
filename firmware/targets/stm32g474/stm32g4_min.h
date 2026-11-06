/* Minimal STM32G474 register definitions for the armfw target layer.
 *
 * ST's CMSIS device header (stm32g474xx.h from STMicroelectronics/cmsis_device_g4) could not be
 * downloaded in the environment this was written in, so this file declares only the registers and
 * bits the target layer uses, with CMSIS-compatible names (RCC->APB2ENR, USART_CR1_UE, ...), so
 * the official header is a drop-in replacement: build with -DARMFW_USE_CMSIS and put ST's
 * headers on the include path.
 *
 * Addresses and bit positions are from RM0440 (STM32G4 reference manual) as recalled, NOT
 * machine-checked against ST's header. VERIFY EVERY LINE AGAINST RM0440 / stm32g474xx.h BEFORE
 * FLASHING. The places most worth double-checking are marked "verify".
 */
#ifndef STM32G4_MIN_H
#define STM32G4_MIN_H

#include <stdint.h>

#define __IO volatile

/* ---------------- core peripherals (ARMv7-M, fixed) ---------------- */
typedef struct {
    __IO uint32_t CTRL, LOAD, VAL, CALIB;
} SysTick_Type;
#define SysTick ((SysTick_Type *)0xE000E010u)

#define NVIC_ISER ((__IO uint32_t *)0xE000E100u)
#define NVIC_ICER ((__IO uint32_t *)0xE000E180u)
#define NVIC_IPR ((__IO uint8_t *)0xE000E400u)
#define SCB_CPACR (*(__IO uint32_t *)0xE000ED88u)
#define SCB_VTOR (*(__IO uint32_t *)0xE000ED08u)
#define SCB_AIRCR (*(__IO uint32_t *)0xE000ED0Cu)
#define DWT_CTRL (*(__IO uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(__IO uint32_t *)0xE0001004u)
#define DEMCR (*(__IO uint32_t *)0xE000EDFCu)
#define DEMCR_TRCENA (1u << 24)

static inline void nvic_enable(unsigned irq, uint8_t prio) {
    NVIC_IPR[irq] = (uint8_t)(prio << 4); /* 4 priority bits implemented */
    NVIC_ISER[irq >> 5] = 1u << (irq & 31u);
}

/* IRQ numbers (RM0440 vector table) - verify */
enum {
    DMA1_Channel1_IRQn = 11,
    DMA1_Channel2_IRQn = 12,
    USART1_IRQn = 37,
    EXTI15_10_IRQn = 40,
    TIM6_DAC_IRQn = 54,
    LPUART1_IRQn = 91,
};

/* ---------------- RCC ---------------- */
typedef struct {
    __IO uint32_t CR;        /* 0x00 */
    __IO uint32_t ICSCR;     /* 0x04 */
    __IO uint32_t CFGR;      /* 0x08 */
    __IO uint32_t PLLCFGR;   /* 0x0C */
    uint32_t RESERVED0[2];   /* 0x10 */
    __IO uint32_t CIER;      /* 0x18 */
    __IO uint32_t CIFR;      /* 0x1C */
    __IO uint32_t CICR;      /* 0x20 */
    uint32_t RESERVED1;      /* 0x24 */
    __IO uint32_t AHB1RSTR;  /* 0x28 */
    __IO uint32_t AHB2RSTR;  /* 0x2C */
    __IO uint32_t AHB3RSTR;  /* 0x30 */
    uint32_t RESERVED2;      /* 0x34 */
    __IO uint32_t APB1RSTR1; /* 0x38 */
    __IO uint32_t APB1RSTR2; /* 0x3C */
    __IO uint32_t APB2RSTR;  /* 0x40 */
    uint32_t RESERVED3;      /* 0x44 */
    __IO uint32_t AHB1ENR;   /* 0x48 */
    __IO uint32_t AHB2ENR;   /* 0x4C */
    __IO uint32_t AHB3ENR;   /* 0x50 */
    uint32_t RESERVED4;      /* 0x54 */
    __IO uint32_t APB1ENR1;  /* 0x58 */
    __IO uint32_t APB1ENR2;  /* 0x5C */
    __IO uint32_t APB2ENR;   /* 0x60 */
    uint32_t RESERVED5[9];   /* 0x64 .. 0x84 */
    __IO uint32_t CCIPR;     /* 0x88 */
    uint32_t RESERVED6;      /* 0x8C */
    __IO uint32_t BDCR;      /* 0x90 */
    __IO uint32_t CSR;       /* 0x94 */
    __IO uint32_t CRRCR;     /* 0x98 */
    __IO uint32_t CCIPR2;    /* 0x9C */
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)0x40021000u)

#define RCC_CR_HSION (1u << 8)
#define RCC_CR_HSIRDY (1u << 10)
#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_PLLON (1u << 24)
#define RCC_CR_PLLRDY (1u << 25)
#define RCC_CRRCR_HSI48ON (1u << 0)
#define RCC_CRRCR_HSI48RDY (1u << 1)
#define RCC_CFGR_SW_PLL (3u << 0)
#define RCC_CFGR_SWS_Msk (3u << 2)
#define RCC_CFGR_SWS_PLL (3u << 2)
#define RCC_CFGR_HPRE_DIV2 (8u << 4)
#define RCC_PLLCFGR_PLLSRC_HSI (2u << 0)
#define RCC_PLLCFGR_PLLM_Pos 4u
#define RCC_PLLCFGR_PLLN_Pos 8u
#define RCC_PLLCFGR_PLLREN (1u << 24)
#define RCC_PLLCFGR_PLLR_Pos 25u
#define RCC_AHB1ENR_DMA1EN (1u << 0)
#define RCC_AHB1ENR_DMAMUX1EN (1u << 2)
#define RCC_AHB2ENR_GPIOAEN (1u << 0)
#define RCC_AHB2ENR_GPIOBEN (1u << 1)
#define RCC_AHB2ENR_GPIOCEN (1u << 2)
#define RCC_AHB2ENR_ADC12EN (1u << 13)
#define RCC_APB1ENR1_TIM2EN (1u << 0)
#define RCC_APB1ENR1_TIM6EN (1u << 4)
#define RCC_APB1ENR1_USBEN (1u << 23)
#define RCC_APB1ENR1_PWREN (1u << 28)
#define RCC_APB1ENR2_LPUART1EN (1u << 0)
#define RCC_APB2ENR_SYSCFGEN (1u << 0)
#define RCC_APB2ENR_USART1EN (1u << 14)

/* ---------------- FLASH / PWR ---------------- */
#define FLASH_ACR (*(__IO uint32_t *)0x40022000u)
#define FLASH_ACR_LATENCY_4WS 4u
#define FLASH_ACR_PRFTEN (1u << 8)
#define FLASH_ACR_ICEN (1u << 9)
#define FLASH_ACR_DCEN (1u << 10)
#define PWR_CR5 (*(__IO uint32_t *)0x40007080u) /* verify: R1MODE bit 8, 0 = boost */
#define PWR_CR5_R1MODE (1u << 8)

/* ---------------- GPIO ---------------- */
typedef struct {
    __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR;
    __IO uint32_t AFR[2];
    __IO uint32_t BRR;
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)0x48000000u)
#define GPIOB ((GPIO_TypeDef *)0x48000400u)
#define GPIOC ((GPIO_TypeDef *)0x48000800u)

/* ---------------- TIM (basic + general purpose subset) ---------------- */
typedef struct {
    __IO uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR;
} TIM_TypeDef;
#define TIM2 ((TIM_TypeDef *)0x40000000u) /* 32-bit */
#define TIM6 ((TIM_TypeDef *)0x40001000u)
#define TIM_CR1_CEN (1u << 0)
#define TIM_CR1_URS (1u << 2)
#define TIM_DIER_UIE (1u << 0)
#define TIM_SR_UIF (1u << 0)
#define TIM_EGR_UG (1u << 0)

/* ---------------- USART / LPUART (same register layout) ---------------- */
typedef struct {
    __IO uint32_t CR1;   /* 0x00 */
    __IO uint32_t CR2;   /* 0x04 */
    __IO uint32_t CR3;   /* 0x08 */
    __IO uint32_t BRR;   /* 0x0C */
    __IO uint32_t GTPR;  /* 0x10 */
    __IO uint32_t RTOR;  /* 0x14 */
    __IO uint32_t RQR;   /* 0x18 */
    __IO uint32_t ISR;   /* 0x1C */
    __IO uint32_t ICR;   /* 0x20 */
    __IO uint32_t RDR;   /* 0x24 */
    __IO uint32_t TDR;   /* 0x28 */
    __IO uint32_t PRESC; /* 0x2C */
} USART_TypeDef;
#define USART1 ((USART_TypeDef *)0x40013800u)
#define LPUART1 ((USART_TypeDef *)0x40008000u)
#define USART_CR1_UE (1u << 0)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_TCIE (1u << 6)
#define USART_CR1_FIFOEN (1u << 29)
#define USART_CR3_DMAR (1u << 6)
#define USART_CR3_DMAT (1u << 7)
#define USART_CR3_HDSEL (1u << 3)
#define USART_ISR_TC (1u << 6)
#define USART_ISR_TXE (1u << 7)
#define USART_ISR_ORE (1u << 3)
#define USART_ICR_TCCF (1u << 6)
#define USART_ICR_ORECF (1u << 3)

/* ---------------- DMA / DMAMUX ---------------- */
typedef struct {
    __IO uint32_t CCR, CNDTR, CPAR, CMAR;
    uint32_t RESERVED;
} DMA_Channel_TypeDef;
typedef struct {
    __IO uint32_t ISR, IFCR;
    DMA_Channel_TypeDef CH[8]; /* channel x at 0x08 + 0x14 * (x - 1) */
} DMA_TypeDef;
#define DMA1 ((DMA_TypeDef *)0x40020000u)
#define DMAMUX1_CCR ((__IO uint32_t *)0x40020800u) /* CxCR, x = 0..15 (DMA1 ch1..8 -> 0..7) */
#define DMA_CCR_EN (1u << 0)
#define DMA_CCR_TCIE (1u << 1)
#define DMA_CCR_DIR (1u << 4)
#define DMA_CCR_CIRC (1u << 5)
#define DMA_CCR_MINC (1u << 7)
#define DMA_CCR_PL_HIGH (2u << 12)
/* DMAMUX request IDs (RM0440 table "DMAMUX: assignment of multiplexer inputs") - verify */
#define DMAMUX_REQ_USART1_RX 24u
#define DMAMUX_REQ_USART1_TX 25u
#define DMAMUX_REQ_LPUART1_RX 34u
#define DMAMUX_REQ_LPUART1_TX 35u

/* ---------------- EXTI / SYSCFG ---------------- */
typedef struct {
    __IO uint32_t IMR1, EMR1, RTSR1, FTSR1, SWIER1, PR1;
} EXTI_TypeDef;
#define EXTI ((EXTI_TypeDef *)0x40010400u)
#define SYSCFG_EXTICR ((__IO uint32_t *)0x40010008u) /* EXTICR1..4 */

#endif
