#include "control.h"
#include "config.h"
#include <math.h>

void control_init(control_t *c)
{
    c->tps1_avg = 0.0f;
    c->tps2_avg = 0.0f;
    c->bse = 0;
    c->fault_ms = UINT32_MAX;
}

static float calibrate(float v, float v0, float v100)
{
    const float x = (v - v0) / (v100 - v0);
    return fmaxf(0.0f, fminf(x, 1.0f));
}

static float iir(float *avg, float x)
{
    *avg = (*avg == 0.0f) ? x : *avg * TPS_IIR_RATIO + x * (1.0f - TPS_IIR_RATIO);
    return *avg;
}

control_throttle_t control_throttle(control_t *c, float tps1_v, float tps2_v)
{
    control_throttle_t p;

    p.tps1 = iir(&c->tps1_avg, calibrate(tps1_v, TPS1_0PER, TPS1_100PER));
    p.tps2 = iir(&c->tps2_avg, calibrate(tps2_v, TPS2_0PER, TPS2_100PER));
    p.tps = (p.tps1 + p.tps2) / 2.0f;

    p.tps1_oor = tps1_v < TPS1_FAULT_LOW || tps1_v > TPS1_FAULT_HIGH;
    p.tps2_oor = tps2_v < TPS2_FAULT_LOW || tps2_v > TPS2_FAULT_HIGH;
    p.tps_disagree = fabsf(p.tps1 - p.tps2) > APPS_TRIP_PERCENT;
    return p;
}

control_brakes_t control_brakes(float bps) {
    control_brakes_t b;

    b.bps = bps;

    /* A shorted sensor must not count as braking for the RTD interlock */
    b.bps_oor = control_bps_oor(bps);
    b.brake_pressed = bps > BPS_SETPOINT_V && !b.bps_oor;

    return b;
}

uint8_t control_bps_oor(float bps_v)
{
    return bps_v < BPS_FAULT_LOW || bps_v > BPS_FAULT_HIGH;
}

float control_bps_psi(float bps_v)
{
    return calibrate(bps_v, BPS_0PSI_V, BPS_FULL_V) * BPS_FULL_PSI;
}

uint8_t control_bse(control_t *c, uint8_t brake, float tps)
{
    if (!c->bse) {
        c->bse = brake && tps >= BSE_TRIP_TPS;
    } else {
        c->bse = tps >= BSE_CLEAR_TPS;
    }
    return c->bse;
}

uint8_t control_fault_debounce(control_t *c, uint8_t fault, uint32_t dt_ms)
{
    /* Checked before counting this step, so trip and clear both lag one step */
    const uint8_t tripped = c->fault_ms > DISABLE_DEBOUNCE_MS;

    if (!fault) {
        c->fault_ms = 0;
    } else if (c->fault_ms <= UINT32_MAX - dt_ms) {
        c->fault_ms += dt_ms;
    }
    return tripped;
}

float control_torque_map(float tps)
{
    return (fmaxf(TMAP_DRIVE_START, fminf(tps, TMAP_DEADBAND_HIGH)) - TMAP_DRIVE_START) *
           (1.0f / (TMAP_DEADBAND_HIGH - TMAP_DRIVE_START));
}

int32_t control_torque_limit(float x, uint32_t rpm, uint32_t dpl_w)
{
    const float w = fmaxf(TORQUE_W_MIN_RAD_S, (float)rpm * RPM_TO_RAD_S);
    const float power_limit = 10.0f * (float)dpl_w * DRIVE_EFFICIENCY / w;

    float limit = fminf((float)TORQUE_LIMIT_NM_X10, power_limit);

    /* The high-speed cap lowers the limit, never raises it past the DPL */
    if (rpm >= TORQUE_HIGH_SPEED_RPM) {
        limit = fminf(TORQUE_HIGH_SPEED_NM_X10, limit);
    }

    return (int32_t)(x * limit);
}

void control_pack_inverter(uint8_t *d, int32_t torque, uint8_t lockout, uint8_t disable,
                           uint8_t heartbeat)
{
    /* Lockout clears only after a disable command, so hold enable low */
    const uint8_t enable = lockout ? 0u : (uint8_t)(~disable & 0x01);

    d[0] = torque & 0xFF;
    d[1] = torque >> 8 & 0xFF;
    d[2] = 0x00;
    d[3] = 0x00;
    d[4] = 0x01; /* Forward */
    d[5] = enable | 0x02 | (uint8_t)(heartbeat << 4);
    d[6] = 0x00;
    d[7] = 0x00;
}

void control_pack_debug(
    uint8_t *buf, 
    const control_data_t *controls, 
    uint8_t rtd_timeout, 
    uint8_t inverter_lockout, 
    uint8_t inverter_enabled,
    uint8_t bse_error,
    uint8_t ready_to_drive,
    uint8_t should_disable_inverter
) {
    buf[0] = rtd_timeout;
    buf[1] = (controls->adcs.bps_adc >> 4) & 0xFF;
    buf[2] = (controls->adcs.tps1_adc >> 4) & 0xFF;
    buf[3] = (controls->adcs.tps2_adc >> 4) & 0xFF;
    buf[4] = (uint8_t)((inverter_lockout << 7) | (inverter_enabled << 6) |
                     (controls->throttle.tps_disagree << 5) | (controls->throttle.tps2_oor << 4) | (controls->throttle.tps1_oor << 3) |
                     (bse_error << 2) | (ready_to_drive << 1) | should_disable_inverter);
    buf[5] = (int)(controls->throttle.tps1 * 100) & 0xFF;
    buf[6] = (int)(controls->throttle.tps2 * 100) & 0xFF;
    buf[7] = (int)(controls->throttle.tps * 100) & 0xFF;
}

void control_pack_bps(uint8_t *d, float psi, uint16_t adc, uint8_t pressed, uint8_t oor)
{
    const uint16_t p = (uint16_t)(psi / BPS_CAN_PSI_PER_BIT + 0.5f);

    d[0] = p & 0xFF;
    d[1] = p >> 8 & 0xFF;
    d[2] = adc & 0xFF;
    d[3] = adc >> 8 & 0xFF;
    d[4] = (uint8_t)((oor << 1) | pressed);
}

static uint16_t sat_u16(int32_t v)
{
    return (v < 0) ? 0u : (v > 0xFFFF) ? 0xFFFFu : (uint16_t)v;
}

void control_pack_limits(uint8_t *d, int32_t drive_x10, int32_t regen_x10, const bms_limits_t *lim)
{
    const uint16_t drive = sat_u16(drive_x10);
    const uint16_t regen = sat_u16(regen_x10);
    const uint16_t dpl = car_limits_0p1kw(lim->dpl_w);

    d[0] = drive & 0xFF;
    d[1] = drive >> 8 & 0xFF;
    d[2] = regen & 0xFF;
    d[3] = regen >> 8 & 0xFF;
    d[4] = dpl & 0xFF;
    d[5] = dpl >> 8 & 0xFF;
    d[6] = (uint8_t)((lim->live ? CONTROL_LIMITS_LIVE : 0) | (lim->limp ? CONTROL_LIMITS_LIMP : 0) |
                     ((lim->why & BMS_LIMITS_WHY_STALE) ? CONTROL_LIMITS_STALE : 0) |
                     ((lim->why & BMS_LIMITS_WHY_VERSION) ? CONTROL_LIMITS_VERSION : 0) |
                     ((lim->why & BMS_LIMITS_WHY_COUNTER) ? CONTROL_LIMITS_COUNTER : 0));
    d[7] = 0x00;
}
