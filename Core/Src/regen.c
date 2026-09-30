#include "regen.h"
#include "config.h"
#include <math.h>

#define RPM_TO_RAD_S (2.0f * 3.14159265f / 60.0f)

static regen_debug_t dbg;

void regen_init(void)
{
    dbg = (regen_debug_t){0};
}

const regen_debug_t *regen_get_debug(void)
{
    return &dbg;
}

static float pedal_factor(float tps)
{
    const float u = (TMAP_REGEN_END - tps) / (TMAP_REGEN_END - TMAP_DEADBAND_LOW);
    const float f = fmaxf(0.0f, fminf(u, 1.0f));

    /* Smoothstep, gentle near foot-off and coast */
    return f * f * (3.0f - 2.0f * f);
}

static float max_regen_nm(uint32_t rpm, float v_pack)
{
    if (rpm <= REGEN_CUTOFF_RPM) {
        return 0.0f;
    }

    const float ramp =
        fminf((float)(rpm - REGEN_CUTOFF_RPM) / (float)(REGEN_RAMP_RPM - REGEN_CUTOFF_RPM), 1.0f);

    const float y = fminf((float)rpm * (100.0f / REGEN_RPM_MAX), 100.0f);
    const float charge_limit =
        REGEN_CELL_CHARGE_A * SOC_KF_NP * v_pack / ((float)rpm * RPM_TO_RAD_S);

    return ramp * fminf((REGEN_PEAK_NM / 100.0f) * (2.0f * y - y * y / 100.0f), charge_limit);
}

static float soc_scale(float soc)
{
    const float u = (REGEN_SOC_OFF - soc) / (REGEN_SOC_OFF - REGEN_SOC_FULL);
    const float f = fminf(fmaxf(u, 0.0f), 1.0f);
    return f * f;
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

int32_t regen_update(int32_t drive_torque, uint32_t motor_speed_rpm, float tps,
                     const soc_kf_debug_t *kf, uint8_t cut, uint32_t dt_ms)
{
    dbg.soc_ok = (kf->flags & SOC_KF_FLAG_INIT) && (kf->flags & SOC_KF_FLAG_BMS_LIVE) &&
                 !(kf->flags & SOC_KF_FLAG_VBAD);
    dbg.soc_scale = dbg.soc_ok ? soc_scale(kf->soc) : 0.0f;
    dbg.cut = cut;

    dbg.pedal = pedal_factor(tps);
    dbg.max_nm = max_regen_nm(motor_speed_rpm, kf->v_pack);

    dbg.target = drive_torque - (int32_t)(10.0f * dbg.soc_scale * dbg.pedal * dbg.max_nm);

    dbg.torque = cut ? drive_torque : slew(dbg.torque, dbg.target, dt_ms);
    return dbg.torque;
}

void regen_pack_debug(uint8_t *d)
{
    const int16_t target = (int16_t)dbg.target;
    const int16_t torque = (int16_t)dbg.torque;

    d[0] = (uint8_t)((dbg.torque < 0) | (dbg.cut << 1) | (dbg.soc_ok << 2) |
                     ((dbg.soc_scale > 0.0f) << 3));
    d[1] = (uint8_t)(dbg.pedal * 100.0f);
    d[2] = (uint8_t)dbg.max_nm;
    d[3] = (uint8_t)(target & 0xFF);
    d[4] = (uint8_t)((target >> 8) & 0xFF);
    d[5] = (uint8_t)(torque & 0xFF);
    d[6] = (uint8_t)((torque >> 8) & 0xFF);
    d[7] = (uint8_t)(dbg.soc_scale * 100.0f);
}
