/* hal_port implementation for STM32G474, register level (LL-style), see board.h for pins.
 *
 * Design:
 *  - time_us: TIM2, 32-bit, prescaled to 1 MHz, free running (wraps every 71.6 min like the core
 *    expects).
 *  - Host link: LPUART1 (bring-up through ST-LINK VCP). RX by DMA1 ch1 in circular mode into a
 *    ring; host_read() consumes from the ring using CNDTR, no per-byte interrupt. TX: host_write()
 *    appends to a staging buffer; staging is kicked to DMA1 ch2 at the end of every control step
 *    (hal_stm32_flush) when the previous TX DMA is done. USB CDC replaces this later
 *    (usb_cdc_glue.c), with the same ring semantics.
 *  - Servo bus: USART1 single-wire half-duplex (HDSEL). servo_txrx(): copy to the TX buffer,
 *    receiver off (no echo), buffer DIR = TX, start DMA1 ch3. The USART TC interrupt flips DIR
 *    back to RX and re-enables the receiver; replies land by DMA1 ch4 (circular) in a ring that
 *    servo_read() drains on the next control step.
 *  - E-stop: PB12 level + EXTI12 falling edge latch, so even a sub-millisecond opening is seen.
 *  - ADC: adc_stm32g4.c (raw codes; scaling in the core). Calibration flash: flash_stm32g4.c.
 *  - Host link: USB CDC (usb_cdc_glue.c) when ARMFW_HOST_LINK_USB is defined, else LPUART1.
 */
#include <string.h>

#include "armfw/hal_port.h"
#include "board.h"
#include "stm32g4_min.h"

/* ---------------- buffers ---------------- */
static uint8_t s_host_rx[BOARD_HOST_RX_RING];
static uint32_t s_host_rx_tail; /* next index to consume */
static uint8_t s_host_tx_dma[BOARD_HOST_TX_BUF];
static uint8_t s_host_tx_stage[BOARD_HOST_TX_BUF];
static uint32_t s_host_tx_len;
static volatile uint8_t s_host_tx_busy;

static uint8_t s_servo_tx[BOARD_SERVO_TX_BUF];
static uint8_t s_servo_rx[BOARD_SERVO_RX_RING];
static uint32_t s_servo_rx_tail;

static volatile uint8_t s_estop_tripped;

/* other target modules (built against ST's CMSIS header) */
void adc_stm32_init(void);
bool adc_stm32_read(adc_raw_t *out);
const flash_port_t *flash_stm32_port(void);
size_t usb_host_read(uint8_t *buf, size_t cap);
size_t usb_host_write(const uint8_t *buf, size_t n);

/* ---------------- helpers ---------------- */
static void gpio_mode(GPIO_TypeDef *g, unsigned pin, unsigned mode, unsigned af) {
    g->MODER = (g->MODER & ~(3u << (2 * pin))) | (mode << (2 * pin));
    if (mode == 2u) {
        __IO uint32_t *afr = &g->AFR[pin >> 3];
        unsigned sh = 4u * (pin & 7u);
        *afr = (*afr & ~(0xFu << sh)) | (af << sh);
        g->OSPEEDR |= 3u << (2 * pin);
    }
}

static void dma_setup(unsigned ch, unsigned req, uint32_t ccr, volatile void *periph, void *mem,
                      uint32_t n) {
    DMA_Channel_TypeDef *c = &DMA1->CH[ch - 1u];
    c->CCR = 0;
    DMAMUX1_CCR[ch - 1u] = req;
    c->CPAR = (uint32_t)(uintptr_t)periph;
    c->CMAR = (uint32_t)(uintptr_t)mem;
    c->CNDTR = n;
    c->CCR = ccr;
}

/* ---------------- init ---------------- */
void hal_stm32_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM6EN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;

    /* DWT cycle counter, used by main for step-time measurement */
    DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;

    /* TIM2: 1 MHz free-running 32-bit timebase */
    TIM2->PSC = BOARD_SYSCLK_HZ / 1000000u - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    /* GPIO: LED PA5 out, DIR PA8 out (0 = RX), e-stop PB12 in with pull-down */
    gpio_mode(GPIOA, 5, 1, 0);
    gpio_mode(GPIOA, 8, 1, 0);
    GPIOA->BRR = 1u << 8;
    gpio_mode(GPIOB, 12, 0, 0);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << 24)) | (2u << 24);
    SYSCFG_EXTICR[3] = (SYSCFG_EXTICR[3] & ~0xFu) | 0x1u; /* EXTI12 <- port B */
    EXTI->FTSR1 |= 1u << 12;
    EXTI->IMR1 |= 1u << 12;
    nvic_enable(EXTI15_10_IRQn, 0);

    /* Host link: LPUART1 PA2/PA3 AF12, 1 Mbaud, DMA RX circular + DMA TX */
    gpio_mode(GPIOA, 2, 2, 12);
    gpio_mode(GPIOA, 3, 2, 12);
    LPUART1->CR1 = 0;
    LPUART1->BRR = (uint32_t)(((uint64_t)BOARD_SYSCLK_HZ * 256u) / BOARD_HOST_BAUD);
    LPUART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    dma_setup(1, DMAMUX_REQ_LPUART1_RX, DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_HIGH | DMA_CCR_EN,
              &LPUART1->RDR, s_host_rx, BOARD_HOST_RX_RING);
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    nvic_enable(DMA1_Channel2_IRQn, 3);

    /* Servo bus: USART1 single-wire half-duplex on PA9, 1 Mbps */
    gpio_mode(GPIOA, 9, 2, 7);
    GPIOA->OTYPER &= ~(1u << 9); /* push-pull into the buffer */
    USART1->CR1 = 0;
    USART1->BRR = BOARD_SYSCLK_HZ / BOARD_SERVO_BAUD; /* oversampling 16 */
    USART1->CR3 = USART_CR3_HDSEL | USART_CR3_DMAR | USART_CR3_DMAT;
    dma_setup(4, DMAMUX_REQ_USART1_RX, DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_HIGH | DMA_CCR_EN,
              &USART1->RDR, s_servo_rx, BOARD_SERVO_RX_RING);
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    nvic_enable(USART1_IRQn, 2);

    adc_stm32_init();
}

/* TIM6: 1 kHz update interrupt drives control_step (see main.c). */
void hal_stm32_start_tick(void) {
    TIM6->PSC = BOARD_SYSCLK_HZ / 1000000u - 1u; /* 1 MHz */
    TIM6->ARR = 1000u - 1u;                       /* 1 kHz */
    TIM6->CR1 = TIM_CR1_URS;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE;
    nvic_enable(TIM6_DAC_IRQn, 1);
    TIM6->CR1 |= TIM_CR1_CEN;
}

/* ---------------- hal_port functions ---------------- */
static uint32_t h_time_us(void *ctx) {
    (void)ctx;
    return TIM2->CNT;
}

static size_t ring_read(const uint8_t *ring, uint32_t size, uint32_t *tail, uint32_t cndtr, uint8_t *buf,
                        size_t cap) {
    uint32_t head = size - cndtr; /* DMA write index */
    if (head >= size) head = 0;
    size_t n = 0;
    while (*tail != head && n < cap) {
        buf[n++] = ring[*tail];
        *tail = (*tail + 1u) % size;
    }
    return n;
}

static size_t h_host_read(void *ctx, uint8_t *buf, size_t cap) {
    (void)ctx;
#ifdef ARMFW_HOST_LINK_USB
    return usb_host_read(buf, cap);
#endif
    /* if the producer laps the consumer (>1 KB in one ms) bytes are lost; the parser resyncs */
    return ring_read(s_host_rx, BOARD_HOST_RX_RING, &s_host_rx_tail, DMA1->CH[0].CNDTR, buf, cap);
}

static size_t h_host_write(void *ctx, const uint8_t *buf, size_t n) {
    (void)ctx;
#ifdef ARMFW_HOST_LINK_USB
    return usb_host_write(buf, n);
#endif
    size_t room = BOARD_HOST_TX_BUF - s_host_tx_len;
    if (n > room) return 0; /* whole frames only */
    memcpy(&s_host_tx_stage[s_host_tx_len], buf, n);
    s_host_tx_len += n;
    return n;
}

/* Called at the end of each control step: hand staged bytes to the TX DMA if it is idle. */
void hal_stm32_flush(void) {
    if (s_host_tx_busy || s_host_tx_len == 0) return;
    memcpy(s_host_tx_dma, s_host_tx_stage, s_host_tx_len);
    uint32_t n = s_host_tx_len;
    s_host_tx_len = 0;
    s_host_tx_busy = 1;
    LPUART1->ICR = USART_ICR_TCCF;
    dma_setup(2, DMAMUX_REQ_LPUART1_TX, DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_EN,
              &LPUART1->TDR, s_host_tx_dma, n);
}

void DMA1_Channel2_IRQHandler(void) {
    DMA1->IFCR = 0xFu << 4; /* clear channel 2 flags */
    DMA1->CH[1].CCR = 0;
    s_host_tx_busy = 0;
}

static void h_servo_txrx(void *ctx, const uint8_t *tx, size_t n) {
    (void)ctx;
    if (n > sizeof s_servo_tx) n = sizeof s_servo_tx;
    memcpy(s_servo_tx, tx, n);
    USART1->CR1 &= ~USART_CR1_RE; /* single wire: do not receive our own bytes */
    GPIOA->BSRR = 1u << 8;        /* buffer direction: TX */
    USART1->ICR = USART_ICR_TCCF;
    dma_setup(3, DMAMUX_REQ_USART1_TX, DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN, &USART1->TDR, s_servo_tx,
              (uint32_t)n);
    USART1->CR1 |= USART_CR1_TCIE;
}

void USART1_IRQHandler(void) {
    if (USART1->ISR & USART_ISR_TC) {
        /* last stop bit is out: release the line within ~1 us so the servo reply is not cut */
        USART1->ICR = USART_ICR_TCCF;
        USART1->CR1 &= ~USART_CR1_TCIE;
        GPIOA->BRR = 1u << 8; /* buffer direction: RX */
        USART1->CR1 |= USART_CR1_RE;
    }
    if (USART1->ISR & USART_ISR_ORE) USART1->ICR = USART_ICR_ORECF;
}

static size_t h_servo_read(void *ctx, uint8_t *buf, size_t cap) {
    (void)ctx;
    return ring_read(s_servo_rx, BOARD_SERVO_RX_RING, &s_servo_rx_tail, DMA1->CH[3].CNDTR, buf, cap);
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & (1u << 12)) {
        EXTI->PR1 = 1u << 12; /* write 1 to clear */
        s_estop_tripped = 1;  /* hardware already cut servo power; firmware records it */
    }
}

static bool h_estop_closed(void *ctx) {
    (void)ctx;
    bool closed = (GPIOB->IDR & (1u << 12)) != 0;
    if (s_estop_tripped) {
        s_estop_tripped = 0;
        return false; /* report a latched opening once, even if the loop already re-closed */
    }
    return closed;
}

static bool h_adc_read(void *ctx, adc_raw_t *out) {
    (void)ctx;
    return adc_stm32_read(out);
}

hal_port_t g_hal_stm32 = {
    .ctx = 0,
    .time_us = h_time_us,
    .host_read = h_host_read,
    .host_write = h_host_write,
    .servo_txrx = h_servo_txrx,
    .servo_read = h_servo_read,
    .estop_loop_closed = h_estop_closed,
    .adc_read = h_adc_read,
    .flash = 0, /* set by hal_stm32_init_flash() before control_init */
};

void hal_stm32_init_flash(void) { g_hal_stm32.flash = flash_stm32_port(); }
