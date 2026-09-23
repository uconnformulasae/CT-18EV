/* 1-RC EKF pack SoC estimator */

#include "soc_kf.h"
#include "config.h"
#include "soc_kf_tables.h"
#include <math.h>

#ifdef SOC_KF_HOST
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

#define ACC_TO_AS 1e-4f
#define BLACKOUT_SOC_PER_S ((SOC_KF_BLACKOUT_I_A / (float)SOC_KF_NP) / (3600.0f * SOC_KF_CAP_AH))

typedef struct {
    /* ISR shared */
    volatile int32_t pending_bms; /* 0.1A * ms */
    volatile uint32_t pending_ms;
    volatile int16_t ibat_raw;
    volatile uint16_t vbat_raw;
    volatile uint8_t btmp_raw;
    volatile uint8_t soc_raw;
    volatile uint32_t last_bms_tick;
    volatile uint32_t bms_dt_tick;
    volatile uint32_t first_bms_tick;
    volatile uint32_t rest_ms;
    volatile uint8_t have_bms;

    /* Filter state */
    float z, v1;
    float p00, p01, p10, p11;
    float charge_ah;
    uint32_t last_update_tick;
    uint8_t initialised;
    uint8_t init_method; /* 1=OCV, 2=Orion */

    soc_kf_debug_t dbg;
} soc_kf_state_t;

static soc_kf_state_t s;

static float lut(const float *tbl, float z, float *slope_out)
{
    const float span = (float)(SOC_KF_N_BP - 1);
    float x = z * span;

    /* Clamp position, not just index: extrapolation sends z to the far rail. */
    if (x < 0.0f)
        x = 0.0f;
    if (x > span)
        x = span;

    int k = (int)x;
    if (k > SOC_KF_N_BP - 2)
        k = SOC_KF_N_BP - 2;

    const float f = x - (float)k;
    const float a = tbl[k];
    const float b = tbl[k + 1];

    if (slope_out) {
        *slope_out = (b - a) * span;
    }
    return a + f * (b - a);
}

static float ocv_inverse(float v)
{
    if (v <= soc_kf_ocv[0])
        return 0.0f;
    if (v >= soc_kf_ocv[SOC_KF_N_BP - 1])
        return 1.0f;

    for (int k = 0; k < SOC_KF_N_BP - 1; k++) {
        if (v <= soc_kf_ocv[k + 1]) {
            const float d = soc_kf_ocv[k + 1] - soc_kf_ocv[k];
            const float f = (d > 0.0f) ? (v - soc_kf_ocv[k]) / d : 0.0f;
            return ((float)k + f) / (float)(SOC_KF_N_BP - 1);
        }
    }
    return 1.0f;
}

static float temp_scale(uint8_t temp_c)
{
    const float t = (float)temp_c;
    if (t <= SOC_KF_TEMP_MIN_C)
        return soc_kf_r_temp[0];
    if (t >= SOC_KF_TEMP_MAX_C)
        return soc_kf_r_temp[SOC_KF_TEMP_N_BP - 1];

    const float x = (t - SOC_KF_TEMP_MIN_C) * 0.2f;
    int k = (int)x;
    if (k > SOC_KF_TEMP_N_BP - 2)
        k = SOC_KF_TEMP_N_BP - 2;

    const float f = x - (float)k;
    return soc_kf_r_temp[k] + f * (soc_kf_r_temp[k + 1] - soc_kf_r_temp[k]);
}

static void clamp_p_diag(void)
{
    if (s.p00 < SOC_KF_P_FLOOR)
        s.p00 = SOC_KF_P_FLOOR;
    if (s.p11 < SOC_KF_P_FLOOR)
        s.p11 = SOC_KF_P_FLOOR;
    if (s.p00 > SOC_KF_P_MAX)
        s.p00 = SOC_KF_P_MAX;
    if (s.p11 > SOC_KF_P_MAX)
        s.p11 = SOC_KF_P_MAX;
}

static int clamp_z(void)
{
    if (s.z < 0.0f) {
        s.z = 0.0f;
        return 1;
    }
    if (s.z > 1.0f) {
        s.z = 1.0f;
        return 1;
    }
    return 0;
}

static int16_t clamp_i16(float v)
{
    if (v > 32767.0f)
        return 32767;
    if (v < -32768.0f)
        return -32768;
    return (int16_t)v;
}

static uint16_t clamp_u16(float v)
{
    if (v > 65535.0f)
        return 65535;
    if (v < 0.0f)
        return 0;
    return (uint16_t)v;
}

void soc_kf_init(void)
{
    const uint32_t pm = crit_enter();

    s.pending_bms = 0;
    s.pending_ms = 0;
    s.ibat_raw = 0;
    s.vbat_raw = 0;
    s.btmp_raw = 0;
    s.soc_raw = 0;
    s.last_bms_tick = 0;
    s.bms_dt_tick = 0;
    s.first_bms_tick = 0;
    s.rest_ms = 0;
    s.have_bms = 0;

    crit_exit(pm);

    s.z = 0.5f;
    s.v1 = 0.0f;
    s.p00 = SOC_KF_P0_Z_OCV;
    s.p01 = 0.0f;
    s.p10 = 0.0f;
    s.p11 = SOC_KF_P0_V1;
    s.charge_ah = 0.0f;
    s.last_update_tick = 0;
    s.initialised = 0;
    s.init_method = 0;

    s.dbg.soc = 0.0f;
    s.dbg.v1 = 0.0f;
    s.dbg.innovation = 0.0f;
    s.dbg.p00 = SOC_KF_P0_Z_OCV;
    s.dbg.p11 = SOC_KF_P0_V1;
    s.dbg.i_pack = 0.0f;
    s.dbg.v_pack = 0.0f;
    s.dbg.charge_ah = 0.0f;
    s.dbg.temp_c = 0;
    s.dbg.orion_soc = 0;
    s.dbg.flags = 0;
}

void soc_kf_feed_bms(int16_t ibat_raw, uint16_t vbat_raw, uint8_t btmp_raw, uint8_t soc_raw,
                     uint32_t tick_ms)
{
#if SOC_KF_IBAT_DISCHARGE_POSITIVE
    const int32_t i_signed = (int32_t)ibat_raw;
#else
    const int32_t i_signed = -(int32_t)ibat_raw;
#endif

    uint32_t dt = 0u;
    if (s.have_bms) {
        dt = tick_ms - s.bms_dt_tick;
    } else {
        s.have_bms = 1;
        s.first_bms_tick = tick_ms;
    }
    s.bms_dt_tick = tick_ms;
    s.last_bms_tick = tick_ms;

    s.ibat_raw = ibat_raw;
    s.vbat_raw = vbat_raw;
    s.btmp_raw = btmp_raw;
    s.soc_raw = soc_raw;

    const int32_t rest_thresh = (int32_t)(SOC_KF_REST_CURRENT_A * 10.0f);
    const int32_t mag = (i_signed < 0) ? -i_signed : i_signed;
    const int gap = (dt > SOC_KF_STALE_MS);

    if (gap || mag >= rest_thresh) {
        s.rest_ms = 0u;
    } else if (dt > 0u && s.rest_ms < 0xFFFF0000u) {
        s.rest_ms += dt;
    }

    if (!gap && dt > 0u) {
        s.pending_bms += i_signed * (int32_t)dt;
        s.pending_ms += dt;
    }
}

void soc_kf_update(uint32_t tick_ms)
{
    const uint32_t pm = crit_enter();
    const int32_t pend_bms = s.pending_bms;
    const uint32_t pend_ms = s.pending_ms;
    const int16_t ibat_raw = s.ibat_raw;
    const uint16_t vbat_raw = s.vbat_raw;
    const uint8_t btmp_raw = s.btmp_raw;
    const uint8_t soc_raw = s.soc_raw;
    const uint32_t bms_tick = s.last_bms_tick;
    const uint32_t first_tick = s.first_bms_tick;
    const uint32_t rest_ms = s.rest_ms;
    const uint8_t have_bms = s.have_bms;
    s.pending_bms = 0;
    s.pending_ms = 0;
    crit_exit(pm);

#if SOC_KF_IBAT_DISCHARGE_POSITIVE
    const float i_pack = (float)ibat_raw * 0.1f;
#else
    const float i_pack = -(float)ibat_raw * 0.1f;
#endif
    const float v_pack = (float)vbat_raw * 0.1f;
    const float i_cell = i_pack / (float)SOC_KF_NP;
    const float v_cell = v_pack / (float)SOC_KF_NS;

    s.charge_ah += (float)pend_bms * ACC_TO_AS / 3600.0f;

    const float dq_cell_as = (float)pend_bms * ACC_TO_AS / (float)SOC_KF_NP;
    if (s.initialised)
        s.z -= dq_cell_as / (3600.0f * SOC_KF_CAP_AH);

    uint8_t flags = 0;
    if (have_bms && (tick_ms - bms_tick) <= SOC_KF_STALE_MS)
        flags |= SOC_KF_FLAG_BMS_LIVE;

    if ((flags & SOC_KF_FLAG_BMS_LIVE) == 0u) {
        flags |= SOC_KF_FLAG_FROZEN;
        if (s.initialised) {
            flags |= SOC_KF_FLAG_INIT;
            if (s.init_method == 1)
                flags |= SOC_KF_FLAG_OCV_INIT;

            const float gap_s = (float)(tick_ms - s.last_update_tick) * 0.001f;
            if (gap_s > 0.0f) {
                s.p00 += (BLACKOUT_SOC_PER_S * BLACKOUT_SOC_PER_S) * gap_s;
                s.p11 += SOC_KF_Q_V1_PER_S * gap_s;
                clamp_p_diag();
            }

            if (clamp_z())
                flags |= SOC_KF_FLAG_CLAMPED;
            s.dbg.soc = s.z;
        }
        s.dbg.charge_ah = s.charge_ah;
        s.dbg.flags = flags;
        s.last_update_tick = tick_ms;
        return;
    }

    const int v_ok = (v_cell >= SOC_KF_VCELL_MIN) && (v_cell <= SOC_KF_VCELL_MAX);
    if (!v_ok)
        flags |= SOC_KF_FLAG_VBAD;

    if (!s.initialised) {
        if (v_ok && rest_ms >= SOC_KF_REST_MS) {
            float slope;
            s.z = ocv_inverse(v_cell);
            (void)lut(soc_kf_ocv, s.z, &slope);
            if (slope < SOC_KF_OCV_SLOPE_MIN)
                slope = SOC_KF_OCV_SLOPE_MIN;

            const float sv = SOC_KF_V1_MAX_V * expf(-(float)rest_ms * 0.001f / SOC_KF_TAU1_S);
            const float gz = sv / slope;

            s.v1 = 0.0f;
            s.initialised = 1;
            s.init_method = 1;
            s.p00 = SOC_KF_P0_Z_OCV + gz * gz;
            s.p01 = gz * sv;
            s.p10 = gz * sv;
            s.p11 = SOC_KF_P0_V1 + sv * sv;
        } else if ((tick_ms - first_tick) >= SOC_KF_INIT_TIMEOUT_MS) {
            s.z = (float)soc_raw * (BMS_SOC_PCT_PER_BIT * 0.01f);
            if (s.z < 0.0f)
                s.z = 0.0f;
            if (s.z > 1.0f)
                s.z = 1.0f;
            s.v1 = 0.0f;
            s.initialised = 1;
            s.init_method = 2;
            s.p00 = SOC_KF_P0_Z_ORION;
            s.p01 = 0.0f;
            s.p10 = 0.0f;
            s.p11 = SOC_KF_P0_V1;
        } else {
            s.dbg.flags = flags;
            s.dbg.i_pack = i_pack;
            s.dbg.v_pack = v_pack;
            s.dbg.orion_soc = soc_raw;
            s.dbg.temp_c = btmp_raw;
            s.last_update_tick = tick_ms;
            return;
        }
        s.last_update_tick = tick_ms;
    }
    flags |= SOC_KF_FLAG_INIT;
    if (s.init_method == 1)
        flags |= SOC_KF_FLAG_OCV_INIT;

    float dt = (float)(tick_ms - s.last_update_tick) * 0.001f;
    if (dt <= 0.0f)
        dt = 0.001f;
    if (dt > 0.5f)
        dt = 0.5f;
    s.last_update_tick = tick_ms;

    const float i_cell_mean = (pend_ms > 0u) ? (dq_cell_as / ((float)pend_ms * 0.001f)) : i_cell;

    const float kt = temp_scale(btmp_raw);
    const float r1 = lut(soc_kf_r1, s.z, 0) * kt;
    const float a = expf(-dt / SOC_KF_TAU1_S);
    s.v1 = a * s.v1 + r1 * (1.0f - a) * i_cell_mean;

    s.p00 = s.p00 + SOC_KF_Q_Z_PER_S * dt;
    s.p01 = a * s.p01;
    s.p10 = a * s.p10;
    s.p11 = a * a * s.p11 + SOC_KF_Q_V1_PER_S * dt;

    float docv_dz;
    const float ocv = lut(soc_kf_ocv, s.z, &docv_dz);
    const float r0 = lut(soc_kf_r0, s.z, 0) * kt;

    const float v_pred = ocv - i_cell * r0 - s.v1;
    const float y = v_cell - v_pred;

    const float h0 = docv_dz;
    const float h1 = -1.0f;

    const float col0 = s.p00 * h0 + s.p01 * h1;
    const float col1 = s.p10 * h0 + s.p11 * h1;

    const float sden = h0 * col0 + h1 * col1 + SOC_KF_R_MEAS;
    if (v_ok && sden > 1e-20f) {
        const float nis = y * y / sden;
        if (nis > SOC_KF_NIS_GATE) {
            flags |= SOC_KF_FLAG_GATED;
            if (rest_ms >= SOC_KF_REST_MS) {
                s.p00 *= SOC_KF_GATE_INFLATE;
                clamp_p_diag();
            }
        } else {
            const float k0 = col0 / sden;
            const float k1 = col1 / sden;

            s.z += k0 * y;
            s.v1 += k1 * y;

            const float r0p0 = h0 * s.p00 + h1 * s.p10;
            const float r0p1 = h0 * s.p01 + h1 * s.p11;
            s.p00 -= k0 * r0p0;
            s.p01 -= k0 * r0p1;
            s.p10 -= k1 * r0p0;
            s.p11 -= k1 * r0p1;
        }
    }

    {
        float m = 0.5f * (s.p01 + s.p10);

        clamp_p_diag();

        const float lim = sqrtf(s.p00 * s.p11);
        if (m > lim)
            m = lim;
        if (m < -lim)
            m = -lim;

        s.p01 = m;
        s.p10 = m;
    }

    if (clamp_z())
        flags |= SOC_KF_FLAG_CLAMPED;

    s.dbg.soc = s.z;
    s.dbg.v1 = s.v1;
    s.dbg.innovation = y;
    s.dbg.p00 = s.p00;
    s.dbg.p11 = s.p11;
    s.dbg.i_pack = i_pack;
    s.dbg.v_pack = v_pack;
    s.dbg.charge_ah = s.charge_ah;
    s.dbg.temp_c = btmp_raw;
    s.dbg.orion_soc = soc_raw;
    s.dbg.flags = flags;
}

const soc_kf_debug_t *soc_kf_get_debug(void)
{
    return &s.dbg;
}

void soc_kf_pack_state(uint8_t *d)
{
    const uint16_t soc = clamp_u16(s.dbg.soc * 10000.0f);
    const int16_t innov = clamp_i16(s.dbg.innovation * 10000.0f);
    const int16_t ah = clamp_i16(s.dbg.charge_ah * 100.0f);

    d[0] = (uint8_t)(soc & 0xFF);
    d[1] = (uint8_t)(soc >> 8);
    d[2] = (uint8_t)((uint16_t)innov & 0xFF);
    d[3] = (uint8_t)((uint16_t)innov >> 8);
    d[4] = (uint8_t)((uint16_t)ah & 0xFF);
    d[5] = (uint8_t)((uint16_t)ah >> 8);
    d[6] = s.dbg.flags;
    d[7] = 0;
}
