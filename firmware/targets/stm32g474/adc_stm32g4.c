/* ADC1 for the board analog front end (pins per hardware/board/out/mcu_pins.md):
 *
 *   rank 1  ADC1_IN1   PA0  ISENSE_ADC  (INA240A2 output)
 *   rank 2  ADC1_IN2   PA1  VSERVO_ADC  (servo rail, 100k/15k)
 *   rank 3  ADC1_IN6   PC0  NTC_ADC     (10k pull-up / NTC 10k B3380)    mcu_pins.md: "verify"
 *   rank 4  ADC1_IN7   PC1  VIN_ADC     (protected 12 V, 100k/15k)        mcu_pins.md: "verify"
 *   rank 5  ADC1_IN18  -    VREFINT     (for the actual VDDA)
 *
 * Continuous scan, hardware oversampling 16x with a 4-bit shift (12-bit result, 16-sample mean),
 * DMA1 channel 5 in circular mode into s_adc[5]. One sequence takes ~0.34 ms (see below), so each
 * 1 kHz control step sees at least two fresh sequences; adc_stm32_read() copies the latest values
 * and reports freshness via the end-of-sequence flag. All scaling is in the portable core
 * (core/src/analog.c).
 *
 * Timing at HCLK/4 = 42.5 MHz ADC clock: inputs 47.5 + 12.5 cycles = 1.41 us x16 = 22.6 us each;
 * VREFINT needs >= 4 us sampling (DS12288 tS_vrefint), 640.5 + 12.5 cycles = 15.4 us x16 = 246 us.
 * Sequence = 4 x 22.6 + 246 = 336 us.
 *
 * Register names come from ST's CMSIS header (third_party/cmsis_device_g4), so addresses and bit
 * positions are ST's. Items NOT covered by that header and written from memory (verify):
 *   - DMAMUX request ID 5 = ADC1 (RM0440 table 91; LL_DMAMUX_REQ_ADC1 in ST's LL headers)
 *   - VREFINT_CAL at 0x1FFF75AA, measured at VDDA = 3.0 V (DS12288 "Internal voltage reference")
 *   - CKMODE = 11 (synchronous HCLK/4) makes RCC CCIPR.ADC12SEL irrelevant
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "armfw/analog.h"
#include "stm32g4xx.h"

#define ADC_N_RANKS 5u
#define DMAMUX_REQ_ADC1 5u              /* verify: RM0440 DMAMUX request table */
#define VREFINT_CAL_ADDR_G4 0x1FFF75AAu /* verify: DS12288 */

static volatile uint16_t s_adc[ADC_N_RANKS];
static uint16_t s_vrefint_cal;
static bool s_adc_ok;

static void delay_cycles(uint32_t n) {
    for (volatile uint32_t i = 0; i < n; i++) {
    }
}

static void pin_analog(GPIO_TypeDef *g, unsigned pin) { g->MODER |= 3u << (2u * pin); }

void adc_stm32_init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    (void)RCC->AHB2ENR;
    pin_analog(GPIOA, 0);
    pin_analog(GPIOA, 1);
    pin_analog(GPIOC, 0);
    pin_analog(GPIOC, 1);

    /* common: synchronous clock HCLK/4, VREFINT path on (both only while ADC1/2 are disabled) */
    ADC12_COMMON->CCR = (3u << ADC_CCR_CKMODE_Pos) | ADC_CCR_VREFEN;

    /* power up: leave deep power-down, start the regulator, wait t_ADCVREG_STUP (20 us) */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    delay_cycles(170u * 25u); /* > 20 us at 170 MHz (a loop iteration is several cycles) */

    /* single-ended calibration */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    uint32_t guard = 1000000u;
    while ((ADC1->CR & ADC_CR_ADCAL) && --guard) {
    }
    if (!guard) return; /* s_adc_ok stays false: the core sees a stale ADC and flags it */
    delay_cycles(100u); /* >= 4 ADC clocks between ADCAL=0 and ADEN=1 */

    /* sequence, sampling, oversampling, DMA circular, continuous */
    ADC1->SMPR1 = (4u << ADC_SMPR1_SMP1_Pos) | (4u << ADC_SMPR1_SMP2_Pos) | (4u << ADC_SMPR1_SMP6_Pos) |
                  (4u << ADC_SMPR1_SMP7_Pos); /* 47.5 cycles */
    ADC1->SMPR2 = 7u << ADC_SMPR2_SMP18_Pos;  /* 640.5 cycles for VREFINT */
    ADC1->SQR1 = ((ADC_N_RANKS - 1u) << ADC_SQR1_L_Pos) | (1u << ADC_SQR1_SQ1_Pos) | (2u << ADC_SQR1_SQ2_Pos) |
                 (6u << ADC_SQR1_SQ3_Pos) | (7u << ADC_SQR1_SQ4_Pos);
    ADC1->SQR2 = 18u << ADC_SQR2_SQ5_Pos;
    ADC1->CFGR2 = ADC_CFGR2_ROVSE | (3u << ADC_CFGR2_OVSR_Pos) | (4u << ADC_CFGR2_OVSS_Pos); /* 16x, >>4 */
    ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_CONT | ADC_CFGR_OVRMOD | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

    /* DMA1 channel 5 <- DMAMUX1 channel 4 <- ADC1 */
    DMA1_Channel5->CCR = 0;
    DMAMUX1_Channel4->CCR = DMAMUX_REQ_ADC1 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    DMA1_Channel5->CPAR = (uint32_t)(uintptr_t)&ADC1->DR;
    DMA1_Channel5->CMAR = (uint32_t)(uintptr_t)s_adc;
    DMA1_Channel5->CNDTR = ADC_N_RANKS;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | (1u << DMA_CCR_PSIZE_Pos) | (1u << DMA_CCR_MSIZE_Pos) |
                         DMA_CCR_PL_0 | DMA_CCR_EN;

    /* enable and start */
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    guard = 1000000u;
    while (!(ADC1->ISR & ADC_ISR_ADRDY) && --guard) {
    }
    if (!guard) return;
    s_vrefint_cal = *(const volatile uint16_t *)VREFINT_CAL_ADDR_G4;
    ADC1->ISR = ADC_ISR_EOS | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    s_adc_ok = true;
}

/* Called from the control step (hal_port.adc_read). */
bool adc_stm32_read(adc_raw_t *out) {
    if (!s_adc_ok) return false;
    if (!(ADC1->ISR & ADC_ISR_EOS)) return false; /* no complete sequence since the last call */
    ADC1->ISR = ADC_ISR_EOS;                       /* write 1 to clear */
    /* The DMA may be part way through the next sequence; each rank is still a complete,
     * oversampled conversion, only from neighbouring sequences (< 0.4 ms apart). */
    out->isense = s_adc[0];
    out->vservo = s_adc[1];
    out->ntc = s_adc[2];
    out->vin = s_adc[3];
    out->vrefint = s_adc[4];
    out->vrefint_cal = s_vrefint_cal;
    return true;
}
