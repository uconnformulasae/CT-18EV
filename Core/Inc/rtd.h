/**
 * @file    rtd.h
 * @brief   Ready-to-drive button debounce.
 *
 * Integrates milliseconds of hold. Holding adds elapsed time, releasing
 * subtracts it scaled by FALL_NUM/DEN. Elapsed time comes from the caller.
 */

#ifndef RTD_H
#define RTD_H

#include <stdint.h>

/* Hold required to latch. */
#define RTD_DEBOUNCE_TRIP_MS 170u

/* Accumulator ceiling. */
#define RTD_DEBOUNCE_MAX_MS 333u

/* Release drains NUM/DEN faster than hold fills, so chatter only gains ground
 * above a NUM/(NUM+DEN) duty cycle, 4/7. */
#define RTD_DEBOUNCE_FALL_NUM 4
#define RTD_DEBOUNCE_FALL_DEN 3

/* Per-call dt is clamped to this. */
#define RTD_DEBOUNCE_DT_MAX_MS 50u

void rtd_init(void);

/* held = RTD button and brake both applied. Returns 1 once the hold passes
 * RTD_DEBOUNCE_TRIP_MS. */
uint8_t rtd_debounce_update(uint8_t held, uint32_t dt_ms);

uint32_t rtd_hold_ms(void);

#endif /* RTD_H */
