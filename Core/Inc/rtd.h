#ifndef RTD_H
#define RTD_H

#include <stdint.h>

#define RTD_DEBOUNCE_TRIP_MS   170u /* Hold time required to latch */
#define RTD_DEBOUNCE_MAX_MS    333u /* Accumulator ceiling */
#define RTD_DEBOUNCE_FALL_NUM  4
#define RTD_DEBOUNCE_FALL_DEN  3
#define RTD_DEBOUNCE_DT_MAX_MS 50u  /* Per-step dt limit */

void rtd_init(void);

/* Update RTD debounce. Returns 1 once latched. */
uint8_t rtd_debounce_update(uint8_t held, uint32_t dt_ms);

uint32_t rtd_hold_ms(void);

#endif /* RTD_H */
