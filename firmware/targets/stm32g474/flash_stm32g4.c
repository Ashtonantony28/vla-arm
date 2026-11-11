/* STM32G474 flash driver for the calibration store (flash_port_t, see core calstore.h).
 *
 * Placement: the last two 2 KB pages of bank 2 in dual-bank mode (option bit DBANK = 1, the
 * factory default on 512 KB parts): page A = bank 2 page 126 at 0x0807F000, page B = bank 2 page
 * 127 at 0x0807F800. The linker script stops the image 4 KB short of the end of flash. The code runs
 * from bank 1, so erasing/programming bank 2 does not stall instruction fetch (read-while-write);
 * this is what lets the core drive the store from the 1 kHz control step.
 *
 * Operations (RM0440 section 3.3.7 / 3.3.8, "Flash program and erase operations"):
 *   unlock: KEYR <- 0x45670123, 0xCDEF89AB (only while CR.LOCK = 1)
 *   erase:  wait !BSY, clear SR error flags, CR = PER | BKER | PNB<<3, then CR |= STRT
 *   program one double word: wait !BSY, clear flags, CR = PG, write the low word then the high
 *           word to the (8-byte aligned) address; the second write starts programming
 *   done:   SR.BSY = 0; errors in SR (PROGERR, WRPERR, PGAERR, SIZERR, PGSERR, MISERR, FASTERR,
 *           OPERR); then CR.PG/PER cleared, caches flushed (stale D-cache lines after an erase)
 *
 * ECC: a double word torn by a power cut can hold an uncorrectable ECC error. Reading it raises
 * ECCD in FLASH->ECCR and an NMI. NMI_Handler below clears ECCD and records the fault if it happened
 * inside flash_read(); flash_read() then reports the region unreadable and the core treats it as
 * garbage. An ECCD outside flash_read() (code or constants) is fatal.
 *
 * Register names/bits are ST's (third_party/cmsis_device_g4). From memory, verify on silicon:
 *   - the two key values above (standard for every STM32 with this flash interface)
 *   - bank 2 base 0x08040000 and 128 x 2 KB pages per bank when DBANK = 1
 *   - that returning from the ECCD NMI resumes the faulting load (data garbage) rather than
 *     re-faulting; if not, flash_read() must instead pre-check with a "safe read" strategy
 *   - the cache flush sequence (DCEN off, DCRST pulse, DCEN on; same for the I-cache)
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "armfw/hal_port.h"
#include "stm32g4xx.h"

#define CAL_BANK2_BASE 0x08040000u /* verify */
#define CAL_PAGE_SIZE 2048u
#define CAL_FIRST_PNB 126u        /* bank 2 page numbers 126, 127 */
#define FLASH_KEY1_G4 0x45670123u /* verify */
#define FLASH_KEY2_G4 0xCDEF89ABu /* verify */
#define FLASH_SR_ERRORS                                                                            \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR |      \
     FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR)

static volatile uint8_t s_in_read;
static volatile uint8_t s_ecc_fault;
static bool s_usable;

static uint32_t page_addr(uint32_t page) { return CAL_BANK2_BASE + (CAL_FIRST_PNB + page) * CAL_PAGE_SIZE; }

static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1_G4;
        FLASH->KEYR = FLASH_KEY2_G4;
    }
}

static void flash_lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

static void flush_caches(void) {
    uint32_t acr = FLASH->ACR;
    if (acr & FLASH_ACR_DCEN) {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
    }
    if (acr & FLASH_ACR_ICEN) {
        FLASH->ACR &= ~FLASH_ACR_ICEN;
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
    }
    FLASH->ACR = acr;
}

static bool f_read(void *ctx, uint32_t page, uint32_t off, uint8_t *buf, uint32_t n) {
    (void)ctx;
    if (!s_usable || page > 1u || off > CAL_PAGE_SIZE || n > CAL_PAGE_SIZE - off) return false;
    if (FLASH->SR & FLASH_SR_BSY) return false; /* bank 2 busy: reads would stall; caller retries */
    FLASH->ECCR = FLASH_ECCR_ECCD | FLASH_ECCR_ECCC; /* write 1 to clear */
    s_ecc_fault = 0;
    s_in_read = 1;
    const volatile uint8_t *src = (const volatile uint8_t *)(uintptr_t)(page_addr(page) + off);
    for (uint32_t i = 0; i < n; i++) buf[i] = src[i];
    __DSB();
    s_in_read = 0;
    bool bad = s_ecc_fault || (FLASH->ECCR & FLASH_ECCR_ECCD);
    FLASH->ECCR = FLASH_ECCR_ECCD | FLASH_ECCR_ECCC;
    return !bad;
}

static bool f_erase_start(void *ctx, uint32_t page) {
    (void)ctx;
    if (!s_usable || page > 1u || (FLASH->SR & FLASH_SR_BSY)) return false;
    flash_unlock();
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_BKER | ((CAL_FIRST_PNB + page) << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    return true;
}

static bool f_program_start(void *ctx, uint32_t page, uint32_t off, const uint8_t dw[8]) {
    (void)ctx;
    if (!s_usable || page > 1u || (off & 7u) || off + 8u > CAL_PAGE_SIZE || (FLASH->SR & FLASH_SR_BSY))
        return false;
    uint32_t lo, hi;
    memcpy(&lo, dw, 4);
    memcpy(&hi, dw + 4, 4);
    flash_unlock();
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PG;
    volatile uint32_t *dst = (volatile uint32_t *)(uintptr_t)(page_addr(page) + off);
    dst[0] = lo;
    __ISB();
    dst[1] = hi; /* starts the double-word program */
    return true;
}

static int f_poll(void *ctx) {
    (void)ctx;
    uint32_t sr = FLASH->SR;
    if (sr & FLASH_SR_BSY) return 0;
    FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_BKER | FLASH_CR_PNB);
    flush_caches();
    flash_lock();
    if (sr & FLASH_SR_ERRORS) {
        FLASH->SR = FLASH_SR_ERRORS;
        return -1;
    }
    FLASH->SR = FLASH_SR_EOP;
    return 1;
}

static const flash_port_t s_port = {
    .ctx = 0,
    .page_size = CAL_PAGE_SIZE,
    .read = f_read,
    .erase_start = f_erase_start,
    .program_start = f_program_start,
    .poll = f_poll,
};

/* Returns the calibration flash port, or NULL when the part is not in dual-bank mode (the store
 * would then sit in the bank the code runs from, and every erase would stall the CPU ~22 ms). */
const flash_port_t *flash_stm32_port(void) {
    s_usable = (FLASH->OPTR & FLASH_OPTR_DBANK) != 0;
    if (s_usable && (FLASH->SR & FLASH_SR_ERRORS)) FLASH->SR = FLASH_SR_ERRORS; /* stale from reset */
    return s_usable ? &s_port : 0;
}

void NMI_Handler(void) {
    if (FLASH->ECCR & FLASH_ECCR_ECCD) {
        FLASH->ECCR = FLASH_ECCR_ECCD;
        if (s_in_read) {
            s_ecc_fault = 1; /* torn calibration double word: flash_read() reports it */
            return;
        }
    }
    for (;;) { /* ECC error in code/constants, clock security system, ...: stop */
    }
}
