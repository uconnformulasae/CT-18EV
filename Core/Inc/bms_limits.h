#ifndef BMS_LIMITS_H
#define BMS_LIMITS_H

#include <stdint.h>

/* The BMS's 0x203 power limits (car_limits.h), with freshness. Without a fresh,
   valid frame the limits are the limp values: DPL CAR_LIMITS_LIMP_DPL_W, CPL 0. */

#define BMS_LIMITS_WHY_STALE   0x01 /* no accepted frame in BMS_LIMITS_STALE_MS */
#define BMS_LIMITS_WHY_VERSION 0x02 /* the last frame was another version */
#define BMS_LIMITS_WHY_COUNTER 0x04 /* the last frame repeated the counter */

typedef struct {
    uint32_t dpl_w; /* discharge power limit, W at the accumulator */
    uint32_t cpl_w; /* charge power limit, W at the accumulator */
    uint8_t live;   /* 1: a fresh, valid frame */
    uint8_t limp;   /* 1: use limp (not live, or the BMS says limp) */
    uint8_t why;    /* BMS_LIMITS_WHY_*, 0 when live */
} bms_limits_t;

void bms_limits_init(void);

/* RX ISR, ID 0x203. tick is a free-running ms count. */
void bms_limits_feed(const uint8_t d[8], uint8_t dlc, uint32_t tick);

/* Copy under a critical section. */
void bms_limits_get(uint32_t tick, bms_limits_t *out);

#endif /* BMS_LIMITS_H */
