#include "control.h"
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

control_pedals_t control_pedals(control_t *c, float tps1_v, float tps2_v)
{
    control_pedals_t p;

    p.tps1 = iir(&c->tps1_avg, calibrate(tps1_v, TPS1_0PER, TPS1_100PER));
    p.tps2 = iir(&c->tps2_avg, calibrate(tps2_v, TPS2_0PER, TPS2_100PER));
    p.tps = (p.tps1 + p.tps2) / 2.0f;

    p.tps1_oor = tps1_v < TPS1_FAULT_LOW || tps1_v > TPS1_FAULT_HIGH;
    p.tps2_oor = tps2_v < TPS2_FAULT_LOW || tps2_v > TPS2_FAULT_HIGH;
    p.tps_disagree = fabsf(p.tps1 - p.tps2) > APPS_TRIP_PERCENT;
    return p;
}

uint8_t control_bps_oor(float bps_v)
{
    return bps_v < BPS_FAULT_LOW || bps_v > BPS_FAULT_HIGH;
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

int32_t control_torque_limit(float x, uint32_t rpm, uint32_t dcl_a)
{
    const float power_limit = (float)(TORQUE_POWER_NUM * dcl_a) /
                              fmaxf(TORQUE_SPEED_DIV_MIN, (float)rpm * TORQUE_RPM_TO_DIV);

    float limit = fminf((float)TORQUE_LIMIT_NM_X10, power_limit);

    if (rpm >= TORQUE_HIGH_SPEED_RPM) {
        limit = TORQUE_HIGH_SPEED_NM_X10;
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

void control_pack_debug(uint8_t *d, const control_status_t *s)
{
    const control_pedals_t *p = &s->pedals;

    d[0] = s->rtd_timeout;
    d[1] = (s->bps_adc >> 4) & 0xFF;
    d[2] = (s->tps1_adc >> 4) & 0xFF;
    d[3] = (s->tps2_adc >> 4) & 0xFF;
    d[4] = (uint8_t)((s->inverter_lockout << 7) | (s->inverter_enabled << 6) |
                     (p->tps_disagree << 5) | (p->tps2_oor << 4) | (p->tps1_oor << 3) |
                     (s->bse << 2) | (s->ready_to_drive << 1) | s->disable);
    d[5] = (int)(p->tps1 * 100) & 0xFF;
    d[6] = (int)(p->tps2 * 100) & 0xFF;
    d[7] = (int)(p->tps * 100) & 0xFF;
}
