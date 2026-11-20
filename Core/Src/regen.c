#include "regen.h"
#include "config.h"
#include <math.h>

static regen_debug_t dbg;

void regen_init(void)
{
    dbg = (regen_debug_t){0};
}

const regen_debug_t *regen_get_debug(void)
{
    return &dbg;
}

#if REGEN_ENABLE
static float pedal_factor(float tps)
{
    const float u = (TMAP_REGEN_END - tps) / (TMAP_REGEN_END - TMAP_DEADBAND_LOW);
    const float f = fmaxf(0.0f, fminf(u, 1.0f));

    /* Smoothstep, gentle near foot-off and coast */
    return f * f * (3.0f - 2.0f * f);
}

static float max_regen_nm(uint32_t rpm, uint32_t cpl_w)
{
    if (rpm <= REGEN_CUTOFF_RPM) {
        return 0.0f;
    }

    const float ramp =
        fminf((float)(rpm - REGEN_CUTOFF_RPM) / (float)(REGEN_RAMP_RPM - REGEN_CUTOFF_RPM), 1.0f);

    const float y = fminf((float)rpm * (100.0f / REGEN_RPM_MAX), 100.0f);
    /* CPL is power at the accumulator; taking it all as shaft power (efficiency 1)
       errs toward less charge current */
    const float charge_limit = (float)cpl_w / ((float)rpm * RPM_TO_RAD_S);

    return ramp * fminf((REGEN_PEAK_NM / 100.0f) * (2.0f * y - y * y / 100.0f), charge_limit);
}

static int32_t slew(int32_t prev, int32_t target, uint32_t dt_ms)
{
    if (dt_ms > REGEN_DT_MAX_MS) {
        dt_ms = REGEN_DT_MAX_MS;
    }

    if (target < prev) {
        const int32_t from = (prev < REGEN_SLEW_BAND) ? prev : REGEN_SLEW_BAND;
        const int32_t lo = from - REGEN_SLEW_DOWN * (int32_t)dt_ms;
        return (target < lo) ? lo : target;
    }

    if (prev < REGEN_SLEW_BAND) {
        const int32_t hi = prev + REGEN_SLEW_UP * (int32_t)dt_ms;
        return (target > hi) ? hi : target;
    }

    return target;
}
#endif

int32_t regen_update(int32_t drive_torque, uint32_t motor_speed_rpm, float tps,
                     const bms_limits_t *lim, uint8_t cut, uint32_t dt_ms)
{
#if !REGEN_ENABLE
    (void)motor_speed_rpm;
    (void)tps;
    (void)lim;
    (void)cut;
    (void)dt_ms;
    dbg.target = drive_torque;
    dbg.torque = drive_torque;
    dbg.cut = 1;
    return drive_torque;
#else
    /* The SoC fade lives in the BMS's CPL; CPL 0 is no regen */
    dbg.limits_live = lim->live;
    dbg.cpl_w = dbg.limits_live ? lim->cpl_w : 0u;
    dbg.cut = cut;

    dbg.pedal = pedal_factor(tps);
    dbg.max_nm = max_regen_nm(motor_speed_rpm, dbg.cpl_w);

    dbg.target = drive_torque - (int32_t)(10.0f * dbg.pedal * dbg.max_nm);

    dbg.torque = cut ? drive_torque : slew(dbg.torque, dbg.target, dt_ms);
    return dbg.torque;
#endif
}

void regen_pack_debug(uint8_t *d)
{
#if !REGEN_ENABLE
    for (int i = 0; i < REGEN_DEBUG_DLC; i++) {
        d[i] = 0;
    }
#else
    const uint32_t cpl_kw = dbg.cpl_w / 1000u;

    d[0] = dbg.cut;
    d[1] = (uint8_t)((cpl_kw > 255u) ? 255u : cpl_kw);
    d[2] = (uint8_t)dbg.max_nm;
#endif
}
