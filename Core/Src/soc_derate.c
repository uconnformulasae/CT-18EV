#include "soc_derate.h"

#define CURVE(...)                                                                         \
    {(const soc_derate_pt_t[]){__VA_ARGS__},                                               \
     sizeof((const soc_derate_pt_t[]){__VA_ARGS__}) / sizeof(soc_derate_pt_t)}

#define SOC_DERATE_TUNE_DEF(name, ...)                                                     \
    __attribute__((unused)) static const soc_derate_tune_t tune_##name = {__VA_ARGS__}

#include "soc_derate_tunes.h"

#define CAT_(a, b) a##b
#define CAT(a, b)  CAT_(a, b)

static const soc_derate_tune_t *const tune = &CAT(tune_, SOC_DERATE_TUNE);

static volatile uint16_t low_ocv_raw;
static volatile uint16_t avg_ocv_raw;
static volatile uint32_t cell_tick;
static volatile uint8_t have_cell;

static uint8_t have_gap;
static soc_derate_debug_t dbg;

static float interp(const soc_derate_curve_t *c, float x)
{
    const soc_derate_pt_t *p = c->pts;
    const uint8_t n = c->n;

    if (x <= p[0].x) {
        return p[0].amps;
    }

    for (uint8_t i = 1; i < n; i++) {
        if (x <= p[i].x) {
            return p[i - 1].amps +
                   (p[i].amps - p[i - 1].amps) * (x - p[i - 1].x) / (p[i].x - p[i - 1].x);
        }
    }

    return p[n - 1].amps;
}

static uint32_t to_amps(float a)
{
    return (a > 0.0f) ? (uint32_t)(a + 0.5f) : 0u;
}

void soc_derate_init(void)
{
    have_cell = 0;
    have_gap = 0;
    dbg = (soc_derate_debug_t){0};
}

void soc_derate_feed_cells(uint16_t low_raw, uint16_t avg_raw, uint32_t tick_ms)
{
    low_ocv_raw = low_raw;
    avg_ocv_raw = avg_raw;
    cell_tick = tick_ms;
    have_cell = 1;
}

/* Gap holds its last value if 0x79F goes stale, imbalance doesn't vanish */
static void update_gap(uint32_t tick_ms, uint32_t dt_ms)
{
    dbg.cell_live = have_cell && (tick_ms - cell_tick) <= SOC_DERATE_CELL_STALE_MS;
    if (!dbg.cell_live) {
        return;
    }

    const float low = soc_kf_ocv_to_soc((float)low_ocv_raw * 1e-4f);
    const float avg = soc_kf_ocv_to_soc((float)avg_ocv_raw * 1e-4f);
    const float gap = (avg > low) ? (avg - low) : 0.0f;

    if (!have_gap) {
        dbg.gap = gap;
        have_gap = 1;
        return;
    }

    float a = (float)dt_ms * 0.001f / SOC_DERATE_GAP_TAU_S;
    if (a > 1.0f) {
        a = 1.0f;
    }
    dbg.gap += a * (gap - dbg.gap);
}

uint32_t soc_derate_dcl(const soc_kf_debug_t *kf, uint32_t bms_dcl, uint32_t tick_ms,
                        uint32_t dt_ms)
{
    update_gap(tick_ms, dt_ms);
    dbg.weak_soc = kf->soc - dbg.gap;

    /* BMS asking for 0 A always wins, otherwise the SoC map owns the limit */
    if (soc_kf_valid(kf) && bms_dcl > 0u) {
        dbg.dcl = to_amps(interp(&tune->soc, dbg.weak_soc));
        dbg.src = SOC_DERATE_SRC_SOC;
    } else {
        dbg.dcl = bms_dcl;
        dbg.src = SOC_DERATE_SRC_BMS;
    }

    return dbg.dcl;
}

const soc_derate_debug_t *soc_derate_get_debug(void)
{
    return &dbg;
}

static uint16_t sat_u16(float v)
{
    return (v <= 0.0f) ? 0u : (v >= 65535.0f) ? 65535u : (uint16_t)(v + 0.5f);
}

void soc_derate_pack_debug(uint8_t *d)
{
    const uint16_t weak = sat_u16(dbg.weak_soc * 10000.0f);
    const uint16_t gap = sat_u16(dbg.gap * 10000.0f);

    d[0] = tune->id;
    d[1] = (uint8_t)(dbg.src | (dbg.cell_live << 1));
    d[2] = (uint8_t)(dbg.dcl & 0xFF);
    d[3] = (uint8_t)(dbg.dcl >> 8);
    d[4] = (uint8_t)(weak & 0xFF);
    d[5] = (uint8_t)(weak >> 8);
    d[6] = (uint8_t)(gap & 0xFF);
    d[7] = (uint8_t)(gap >> 8);
}
