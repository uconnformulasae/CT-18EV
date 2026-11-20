#include "bms_limits.h"

#include "car_limits.h"
#include "config.h"

#ifdef BMS_LIMITS_HOST
static inline uint32_t crit_enter(void)
{
    return 0u;
}
static inline void crit_exit(uint32_t s)
{
    (void)s;
}
#else
#include "stm32f1xx_hal.h"
static inline uint32_t crit_enter(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}
static inline void crit_exit(uint32_t s)
{
    __set_PRIMASK(s);
}
#endif

static struct {
    car_limits_t lim;
    uint32_t tick; /* of the last accepted frame */
    uint8_t have;  /* an accepted frame exists */
    uint8_t why;   /* VERSION/COUNTER of the last rejected frame */
} s;

void bms_limits_init(void)
{
    const uint32_t pm = crit_enter();
    s.lim.dpl_w = CAR_LIMITS_LIMP_DPL_W;
    s.lim.cpl_w = 0u;
    s.lim.flags = CAR_LIMITS_FLAG_LIMP;
    s.lim.counter = 0u;
    s.tick = 0u;
    s.have = 0u;
    s.why = 0u;
    crit_exit(pm);
}

void bms_limits_feed(const uint8_t d[8], uint8_t dlc, uint32_t tick)
{
    if (dlc != CAR_LIMITS_DLC) {
        return;
    }
    car_limits_t l;
    if (!car_limits_unpack(d, &l)) {
        s.why = BMS_LIMITS_WHY_VERSION;
        return;
    }
    if (s.have && l.counter == s.lim.counter) {
        s.why = BMS_LIMITS_WHY_COUNTER; /* a stuck sender does not keep us live */
        return;
    }
    s.lim = l;
    s.tick = tick;
    s.have = 1u;
    s.why = 0u;
}

void bms_limits_get(uint32_t tick, bms_limits_t *out)
{
    const uint32_t pm = crit_enter();
    const car_limits_t l = s.lim;
    /* Signed difference survives the tick wrap. A frame stamped after the caller
       sampled its tick (the RX ISR ran in between) is age 0, not 2^32 - 1. */
    const int32_t diff = (int32_t)(tick - s.tick);
    const uint32_t age = (diff < 0) ? 0u : (uint32_t)diff;
    const uint8_t have = s.have;
    const uint8_t why = s.why;
    crit_exit(pm);

    if (have && age <= BMS_LIMITS_STALE_MS) {
        out->dpl_w = l.dpl_w;
        out->cpl_w = l.cpl_w;
        out->live = 1u;
        out->limp = (uint8_t)((l.flags & CAR_LIMITS_FLAG_LIMP) != 0u);
        out->why = 0u;
    } else {
        out->dpl_w = CAR_LIMITS_LIMP_DPL_W;
        out->cpl_w = 0u;
        out->live = 0u;
        out->limp = 1u;
        out->why = (uint8_t)(BMS_LIMITS_WHY_STALE | why);
    }
}
