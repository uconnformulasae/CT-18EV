/* 0x203 BMS_Power_Limits: the one frame the LVCU reads from the BMS, sent
   every 20 ms on CAN1 (docs/can-protocol.md).

   Identical copy in CT-18EV Core/Inc/car_limits.h; change both. A layout
   change bumps CAR_LIMITS_VERSION, so a mismatched pair falls back to limp.

   Bytes, little-endian: 0-1 DPL, 2-3 CPL (0.1 kW/bit, rounded down), 4 flags,
   5 reserved (0), 6 counter (+1 every frame, wraps), 7 version. */
#ifndef CAR_LIMITS_H
#define CAR_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#define CAR_LIMITS_ID      0x203U
#define CAR_LIMITS_DLC     8U
#define CAR_LIMITS_VERSION 1U

/* What the LVCU uses without fresh, valid limits, and what the BMS sends when
   its own inputs are not trustworthy: the old 125 A at 420 V, regen off
   (charge power 0). */
#define CAR_LIMITS_LIMP_DPL_W 52500U

/* Byte 4. Other bits are 0. */
#define CAR_LIMITS_FLAG_LIMP 0x01U /* the BMS is sending limp values */

typedef struct {
    uint32_t dpl_w;  /* discharge power limit, W at the accumulator */
    uint32_t cpl_w;  /* charge power limit, W at the accumulator */
    uint8_t flags;   /* CAR_LIMITS_FLAG_* */
    uint8_t counter; /* +1 every frame, wraps */
} car_limits_t;

/* W to the frame's 0.1 kW, rounded down and saturated. */
static inline uint16_t car_limits_0p1kw(uint32_t w)
{
    const uint32_t v = w / 100U;
    return (v > 0xFFFFU) ? (uint16_t)0xFFFFU : (uint16_t)v;
}

static inline void car_limits_pack(const car_limits_t *l, uint8_t d[8])
{
    const uint16_t dpl = car_limits_0p1kw(l->dpl_w);
    const uint16_t cpl = car_limits_0p1kw(l->cpl_w);
    d[0] = (uint8_t)(dpl & 0xFFU);
    d[1] = (uint8_t)(dpl >> 8);
    d[2] = (uint8_t)(cpl & 0xFFU);
    d[3] = (uint8_t)(cpl >> 8);
    d[4] = (uint8_t)(l->flags & CAR_LIMITS_FLAG_LIMP);
    d[5] = 0U;
    d[6] = l->counter;
    d[7] = (uint8_t)CAR_LIMITS_VERSION;
}

/* False, leaving *l untouched, for a frame of another version. */
static inline bool car_limits_unpack(const uint8_t d[8], car_limits_t *l)
{
    if (d[7] != (uint8_t)CAR_LIMITS_VERSION) {
        return false;
    }
    l->dpl_w = ((uint32_t)d[0] | ((uint32_t)d[1] << 8)) * 100U;
    l->cpl_w = ((uint32_t)d[2] | ((uint32_t)d[3] << 8)) * 100U;
    l->flags = d[4];
    l->counter = d[6];
    return true;
}

#endif /* CAR_LIMITS_H */
