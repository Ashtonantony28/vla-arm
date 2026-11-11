/* Fake two-page NOR flash with STM32G4-like rules and power-cut fault injection.
 *
 * Rules modelled: erase sets a whole page to 0xFF; programming is in aligned 8-byte double words
 * and only into an erased double word (otherwise PROGERR, no change); a double word torn by a
 * power cut can read back as garbage, garbage with an ECC double error (read fails), or blank.
 *
 * Power cut: `budget` counts programmed bytes (an erase costs FF_ERASE_COST). When an operation
 * would exceed it, that operation is torn at the remaining byte offset, the flash is "dead" and
 * every later operation fails until fake_flash_power_on() (= reboot). budget < 0: unlimited. */
#ifndef ARMFW_FAKE_FLASH_H
#define ARMFW_FAKE_FLASH_H

#include "armfw/hal_port.h"

#define FF_PAGE_SIZE 2048u
#define FF_PAGES 2u
#define FF_DW_PER_PAGE (FF_PAGE_SIZE / 8u)
#define FF_ERASE_COST 8

typedef enum {
    FF_TORN_GARBAGE = 0, /* partially programmed bits, reads without error */
    FF_TORN_ECC = 1,     /* reads fail (ECC double error -> NMI on the real part) */
    FF_TORN_BLANK = 2,   /* looks still erased (marginal cells) */
    FF_TORN_MODES = 3,
} ff_torn_mode_t;

typedef struct {
    uint8_t mem[FF_PAGES][FF_PAGE_SIZE];
    uint8_t ecc_bad[FF_PAGES][FF_DW_PER_PAGE];
    /* operation in progress */
    int result;          /* poll result once not busy */
    uint32_t busy_left;
    uint32_t busy_polls; /* latency: polls returning busy per operation (default 0) */
    /* fault injection */
    int32_t budget;
    bool dead;
    ff_torn_mode_t torn_mode;
    int32_t erase_tear_dw;      /* torn erase: double words [0, n) erased, n is garbage; -1 = random */
    int32_t fail_program_nth;   /* the n-th program (0-based) reports PROGERR; -1 = never */
    int32_t corrupt_program_nth; /* the n-th program flips a bit but reports success; -1 = never */
    uint32_t rng;
    /* statistics */
    uint32_t programs;
    uint32_t erases[FF_PAGES];
    uint32_t reads;
} fake_flash_t;

void fake_flash_init(fake_flash_t *f); /* blank flash, no faults */
void fake_flash_power_on(fake_flash_t *f); /* reboot: clear dead/budget, keep contents */
void fake_flash_port(fake_flash_t *f, flash_port_t *port);

#endif
