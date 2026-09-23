#include "rtd.h"

static int32_t hold_scaled; /* Scaled by RTD_DEBOUNCE_FALL_DEN */

#define HOLD_MAX_SCALED  ((int32_t)RTD_DEBOUNCE_MAX_MS * RTD_DEBOUNCE_FALL_DEN)
#define HOLD_TRIP_SCALED ((int32_t)RTD_DEBOUNCE_TRIP_MS * RTD_DEBOUNCE_FALL_DEN)

void rtd_init(void)
{
    hold_scaled = 0;
}

uint8_t rtd_debounce_update(uint8_t held, uint32_t dt_ms)
{
    if (dt_ms > RTD_DEBOUNCE_DT_MAX_MS) {
        dt_ms = RTD_DEBOUNCE_DT_MAX_MS;
    }

    if (held) {
        hold_scaled += (int32_t)dt_ms * RTD_DEBOUNCE_FALL_DEN;
    } else {
        hold_scaled -= (int32_t)dt_ms * RTD_DEBOUNCE_FALL_NUM;
    }

    if (hold_scaled < 0) {
        hold_scaled = 0;
    }
    if (hold_scaled > HOLD_MAX_SCALED) {
        hold_scaled = HOLD_MAX_SCALED;
    }

    return (hold_scaled > HOLD_TRIP_SCALED) ? 1u : 0u;
}

uint32_t rtd_hold_ms(void)
{
    return (uint32_t)(hold_scaled / RTD_DEBOUNCE_FALL_DEN);
}
