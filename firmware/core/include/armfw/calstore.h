/* Calibration store: armfw_config_t persisted in two flash pages (A/B), power-loss safe.
 *
 * Layout: each page is an append-only log of fixed 96-byte slots (21 slots in a 2 KB page).
 *
 *   off  size  field
 *     0     4  magic 0x4C414341 ("ACAL", little-endian)
 *     4     2  format version (1)
 *     6     2  payload length (76)
 *     8     4  sequence number (monotonic over the life of the board)
 *    12    76  payload: 6 x {i16 offset_ticks, i8 direction, u8 0, i16 lim_min, i16 lim_max,
 *                            i16 vmax, i16 current_limit} + u16 wd_hold_ms + u16 wd_off_ms
 *    88     4  zero padding
 *    92     4  CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF) of bytes 0..91
 *
 * A slot is valid when magic, version, length and CRC match AND every field passes the same
 * range checks as a CONFIG message. Boot picks the valid slot with the highest sequence number in
 * either page; if there is none, defaults are used.
 *
 * Save: the next blank slot after the newest record in the active page; when the active page is
 * full, the other page is erased and written from slot 0. The page holding the newest valid record
 * is never erased or overwritten, so a power cut at any point (mid-program, mid-erase) leaves the
 * previous record readable: boot then falls back to it (or to the other page, or to defaults if
 * nothing was ever saved). Every save is read back and compared; a slot that fails is skipped
 * ("burned") and the next one tried, up to CALSTORE_MAX_RETRIES.
 *
 * Wear: one page erase per 21 saves, alternating pages -> each page is erased once every 42 saves
 * (10 k cycle endurance -> ~420 k saves).
 *
 * The save is incremental: calstore_service() does at most one flash operation start or poll per
 * call, so it can run from the 1 kHz control step without blocking (page erase ~22 ms and double
 * word programming ~85 us run in the background on the target's second flash bank).
 */
#ifndef ARMFW_CALSTORE_H
#define ARMFW_CALSTORE_H

#include "armfw/common.h"
#include "armfw/config.h"
#include "armfw/hal_port.h"

#define CALSTORE_MAGIC 0x4C414341u
#define CALSTORE_VERSION 1u
#define CALSTORE_PAYLOAD_LEN 76u
#define CALSTORE_SLOT_LEN 96u
#define CALSTORE_CRC_OFF 92u
#define CALSTORE_MAX_RETRIES 3u
#define CALSTORE_MAX_POLLS 200u /* an operation still busy after this many service calls = error */

typedef enum {
    CS_IDLE = 0,
    CS_ERASE_WAIT,
    CS_PROGRAM,
    CS_PROGRAM_WAIT,
    CS_VERIFY,
} calstore_state_t;

typedef struct {
    const flash_port_t *fp;
    uint32_t slots_per_page;
    int active_page;   /* page holding the newest valid record, -1 = none */
    uint32_t next_slot; /* first slot after the last used one in active_page */
    uint32_t seq;       /* highest sequence number seen or used */
    /* save in flight */
    calstore_state_t state;
    int target_page;
    uint32_t target_slot;
    uint32_t dw;
    uint32_t polls;
    uint32_t retries;
    bool fail_pending;
    uint8_t rec[CALSTORE_SLOT_LEN];
    /* statistics */
    uint32_t loaded_seq;     /* sequence of the record used at boot, 0 = defaults */
    uint32_t valid_records;  /* found at boot */
    uint32_t garbage_slots;  /* non-blank, invalid slots found at boot */
    uint32_t saves_ok, saves_failed, burned_slots;
    uint32_t erases[2];
} calstore_t;

/* CRC-32/ISO-HDLC ("crc32" of zlib): check value of "123456789" is 0xCBF43926. */
uint32_t calstore_crc32(const uint8_t *d, size_t n);

/* Record (de)serialisation. decode returns false if anything is invalid. */
void calstore_encode(const armfw_config_t *cfg, uint32_t seq, uint8_t out[CALSTORE_SLOT_LEN]);
bool calstore_decode(const uint8_t rec[CALSTORE_SLOT_LEN], armfw_config_t *cfg, uint32_t *seq);

/* Scan both pages and fill cfg with the newest valid record (returns true) or defaults (false).
 * fp may be NULL (no persistence: defaults). Never writes to flash. */
bool calstore_init(calstore_t *cs, const flash_port_t *fp, armfw_config_t *cfg);

/* Start saving cfg (snapshot taken now). Returns 0 if started, -1 if a save is already running,
 * -2 if there is no flash. */
int calstore_save_start(calstore_t *cs, const armfw_config_t *cfg);
/* Advance a running save. Returns 1 when the save just completed and verified, -1 when it just
 * failed, 0 otherwise (still running or idle). */
int calstore_service(calstore_t *cs);
static inline bool calstore_busy(const calstore_t *cs) { return cs->state != CS_IDLE; }

#endif
