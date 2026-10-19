/* Host tests for soc_kf */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../Core/Src/soc_kf.c"

static int failures = 0;
static int checks = 0;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (fabsf(got - want) > tol) {
        failures++;
        printf("  FAIL: %s (got %.6f want %.6f tol %.6f)\n", what, got, want, tol);
    }
}

typedef struct {
    float z, v1;
} cell_t;

static void cell_step(cell_t *c, float i_cell, float dt)
{
    const float r1 = lut(soc_kf_r1, c->z, 0);
    const float a = expf(-dt / SOC_KF_TAU1_S);
    c->z -= i_cell * dt / (3600.0f * SOC_KF_CAP_AH);
    c->v1 = a * c->v1 + r1 * (1.0f - a) * i_cell;
}

static float cell_terminal(const cell_t *c, float i_cell)
{
    return lut(soc_kf_ocv, c->z, 0) - i_cell * lut(soc_kf_r0, c->z, 0) - c->v1;
}

static void test_lut_breakpoints(void)
{
    printf("test_lut_breakpoints\n");
    for (int k = 0; k < SOC_KF_N_BP; k++) {
        const float z = (float)k / (float)(SOC_KF_N_BP - 1);
        check_near(lut(soc_kf_ocv, z, 0), soc_kf_ocv[k], 1e-5f, "ocv at breakpoint");
        check_near(lut(soc_kf_r0, z, 0), soc_kf_r0[k], 1e-7f, "r0 at breakpoint");
    }
    const float span = (float)(SOC_KF_N_BP - 1);
    const float zmid = (20.0f + 0.5f) / span;
    check_near(lut(soc_kf_ocv, zmid, 0), 0.5f * (soc_kf_ocv[20] + soc_kf_ocv[21]), 1e-5f,
               "ocv midpoint");

    check_near(lut(soc_kf_ocv, -1.0f, 0), soc_kf_ocv[0], 1e-5f, "ocv clamps low");
    check_near(lut(soc_kf_ocv, 2.0f, 0), soc_kf_ocv[SOC_KF_N_BP - 1], 1e-5f, "ocv clamps high");

    float slope;
    (void)lut(soc_kf_ocv, zmid, &slope);
    check_near(slope, (soc_kf_ocv[21] - soc_kf_ocv[20]) * span, 1e-3f, "dOCV/dz");
}

static void test_ocv_inverse(void)
{
    printf("test_ocv_inverse\n");
    for (int k = 0; k < SOC_KF_N_BP; k++) {
        const float z = (float)k / (float)(SOC_KF_N_BP - 1);
        check_near(ocv_inverse(soc_kf_ocv[k]), z, 1e-4f, "inverse round-trips");
    }
    check_near(ocv_inverse(1.0f), 0.0f, 1e-6f, "below table -> 0");
    check_near(ocv_inverse(9.9f), 1.0f, 1e-6f, "above table -> 1");
}

/* Returns final |error| in SoC */
static float run_sim(float true_z0, float orion_soc_pct, float i_pack_a, float seconds,
                     int rest_start, float *out_est)
{
    cell_t cell = {true_z0, 0.0f};
    soc_kf_init();

    uint32_t t = 0;
    uint32_t last_update = 0;

    const uint32_t rest_end = rest_start ? 2500u : 0u;

    while (t < (uint32_t)(seconds * 1000.0f)) {
        const float i_pack = (t < rest_end) ? 0.0f : i_pack_a;
        const float i_cell = i_pack / (float)SOC_KF_NP;

        cell_step(&cell, i_cell, 0.020f);
        const float v_cell = cell_terminal(&cell, i_cell);

        const int16_t ibat = (int16_t)lrintf(i_pack * 10.0f);
        const uint16_t vbat = (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f);
        soc_kf_feed_bms(ibat, vbat, 25, (uint8_t)lrintf(orion_soc_pct * 2.0f), t);

        if (t - last_update >= 90u) {
            soc_kf_update(t);
            last_update = t;
        }
        t += 20u;
    }
    if (out_est)
        *out_est = s.dbg.soc;
    return fabsf(s.dbg.soc - cell.z);
}

static void test_ocv_init_at_rest(void)
{
    printf("test_ocv_init_at_rest\n");
    float est = 0.0f;
    const float err = run_sim(0.72f, 50.0f, 40.0f, 30.0f, 1, &est);
    check((s.dbg.flags & SOC_KF_FLAG_OCV_INIT) != 0, "used rest-OCV init");
    check((s.dbg.flags & SOC_KF_FLAG_INIT) != 0, "reports initialised");
    printf("    true->est after 30s: err = %.4f (%.2f%% SoC)\n", err, err * 100.0f);
    check(err < 0.01f, "rest-OCV init tracks within 1% SoC");
}

static void test_converges_from_bad_seed(void)
{
    printf("test_converges_from_bad_seed\n");
    float est = 0.0f;
    const float err = run_sim(0.72f, 50.0f, 40.0f, 120.0f, 0, &est);
    printf("    est=%.4f after 120s, err = %.4f (%.2f%% SoC)\n", est, err, err * 100.0f);
    check(err < 0.03f, "converges to within 3% SoC from a 22% seed error");
}

static void test_discharge_lowers_soc(void)
{
    printf("test_discharge_lowers_soc\n");
    float est = 0.0f;
    run_sim(0.80f, 80.0f, 100.0f, 60.0f, 1, &est);
    check(est < 0.80f, "positive (discharge) current reduces SoC");
    printf("    80%% -> %.2f%% after 60s at 100A\n", est * 100.0f);

    run_sim(0.50f, 50.0f, -100.0f, 60.0f, 1, &est);
    check(est > 0.50f, "negative (regen) current raises SoC");
    printf("    50%% -> %.2f%% after 60s at -100A\n", est * 100.0f);
}

static void test_coulomb_accuracy(void)
{
    printf("test_coulomb_accuracy\n");
    float est = 0.0f;
    run_sim(0.80f, 80.0f, 100.0f, 62.5f, 1, &est);
    const float expected_drop = 100.0f * 60.0f / 3600.0f / (SOC_KF_CAP_AH * (float)SOC_KF_NP);
    printf("    charge_ah = %.4f, expected drop %.4f SoC\n", s.dbg.charge_ah, expected_drop);
    check_near(s.dbg.charge_ah, 100.0f * 60.0f / 3600.0f, 0.05f,
               "cumulative charge matches integral");
}

static void test_staleness_freezes(void)
{
    printf("test_staleness_freezes\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 50.0f, 20.0f, 1, &est);

    /* Stale silence must not move estimate */
    soc_kf_update(19980u);
    const float before = s.dbg.soc;

    uint32_t t = 20000u + SOC_KF_STALE_MS + 500u;
    soc_kf_update(t);
    check((s.dbg.flags & SOC_KF_FLAG_FROZEN) != 0, "sets FROZEN on stale input");
    check((s.dbg.flags & SOC_KF_FLAG_BMS_LIVE) == 0, "clears BMS_LIVE");
    check_near(s.dbg.soc, before, 1e-6f, "estimate held, silence not integrated");
}

static void test_packing(void)
{
    printf("test_packing\n");
    float est = 0.0f;
    run_sim(0.75f, 75.0f, 30.0f, 20.0f, 1, &est);

    uint8_t d[8];
    soc_kf_pack_state(d);

    const uint16_t soc = (uint16_t)(d[0] | (d[1] << 8));
    check_near((float)soc / 10000.0f, s.dbg.soc, 1e-4f, "SoC round-trips");

    const int16_t innov = (int16_t)(uint16_t)(d[2] | (d[3] << 8));
    check_near((float)innov / 10000.0f, s.dbg.innovation, 1e-4f, "innovation round-trips");

    const int16_t ah = (int16_t)(uint16_t)(d[4] | (d[5] << 8));
    check_near((float)ah / 100.0f, s.dbg.charge_ah, 0.01f, "cumulative charge round-trips");

    check(d[6] == s.dbg.flags, "flags in byte 6");
    check((d[6] & SOC_KF_FLAG_INIT) != 0, "INIT reported");
    check(d[7] == 0, "byte 7 zeroed for the caller");
}

static void test_clamping(void)
{
    printf("test_clamping\n");
    float est = 0.0f;
    /* Discharge hits floor */
    run_sim(0.15f, 15.0f, 100.0f, 200.0f, 1, &est);
    printf("    final soc = %.4f, flags = 0x%02X\n", s.dbg.soc, s.dbg.flags);
    check(s.dbg.soc >= 0.0f, "SoC never goes negative");
    check(s.dbg.soc <= 1.0f, "SoC never exceeds 1");
    check(s.dbg.soc < 0.02f, "sustained discharge drives SoC to the floor");
    check((s.dbg.flags & SOC_KF_FLAG_CLAMPED) != 0, "reports the clamp");
}

static void test_rejects_bad_voltage(void)
{
    printf("test_rejects_bad_voltage\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 0.0f, 10.0f, 1, &est);
    const float before = s.dbg.soc;
    check((s.dbg.flags & SOC_KF_FLAG_VBAD) == 0, "healthy voltage is accepted");

    /* Implausible voltage rejected */
    uint32_t t = 10000u;
    for (int k = 0; k < 50; k++) {
        soc_kf_feed_bms(0, 65000u, 25, 60, t);
        if (k % 5 == 0)
            soc_kf_update(t);
        t += 20u;
    }
    printf("    soc %.4f -> %.4f under garbage voltage\n", before, s.dbg.soc);
    check((s.dbg.flags & SOC_KF_FLAG_VBAD) != 0, "flags implausible voltage");
    check_near(s.dbg.soc, before, 1e-3f, "estimate unmoved by garbage voltage");
}

static void test_orion_seed_scaling(void)
{
    printf("test_orion_seed_scaling\n");
    cell_t cell = {0.75f, 0.0f};
    soc_kf_init();

    uint32_t t = 0u, last = 0u;
    float z_at_seed = -1.0f;
    while (t < SOC_KF_INIT_TIMEOUT_MS + 500u) {
        const float i_cell = 100.0f / (float)SOC_KF_NP;
        cell_step(&cell, i_cell, 0.020f);
        const float v_cell = cell_terminal(&cell, i_cell);
        soc_kf_feed_bms(1000, (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 150, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            last = t;
        }
        if (s.initialised && z_at_seed < 0.0f)
            z_at_seed = s.z;
        t += 20u;
    }
    check(s.init_method == 2, "no rest -> Orion seed");
    printf("    raw 150 seeded z = %.4f\n", z_at_seed);
    check_near(z_at_seed, 0.75f, 0.02f, "Orion SoC is 0.5%/bit");
}

static float run_then_garbage(int n, int *gated)
{
    cell_t cell = {0.60f, 0.0f};
    soc_kf_init();
    uint32_t t = 0u, last = 0u;
    *gated = 0;
    for (; t < 40000u; t += 20u) {
        const float i_pack = (t < 2500u) ? 0.0f : 60.0f;
        const float i_cell = i_pack / (float)SOC_KF_NP;
        cell_step(&cell, i_cell, 0.020f);
        const float v_cell = cell_terminal(&cell, i_cell);
        soc_kf_feed_bms((int16_t)lrintf(i_pack * 10.0f),
                        (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 120, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            last = t;
        }
    }
    for (int k = 0; k < n; k++) {
        cell_step(&cell, 15.0f, 0.020f);
        soc_kf_feed_bms(600, 3200u, 25, 120, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            if (s.dbg.flags & SOC_KF_FLAG_GATED)
                *gated = 1;
            last = t;
        }
        t += 20u;
    }
    return fabsf(s.dbg.soc - cell.z);
}

static void test_in_range_garbage_is_gated(void)
{
    printf("test_in_range_garbage_is_gated\n");
    int gated = 0;
    const float err = run_then_garbage(250, &gated);
    printf("    250 garbage frames -> err %.2f%% SoC, gated=%d\n", err * 100.0f, gated);
    check((s.dbg.flags & SOC_KF_FLAG_VBAD) == 0, "VBAD cannot see an in-range value");
    check(gated, "the innovation gate can");
    check(err < 0.02f, "estimate holds against sustained in-range garbage");

    uint8_t d[8];
    soc_kf_pack_state(d);
    check((d[6] & SOC_KF_FLAG_GATED) != 0, "GATED is visible on CAN");
}

static void test_gated_filter_recovers_at_rest(void)
{
    printf("test_gated_filter_recovers_at_rest\n");
    cell_t cell = {0.30f, 0.0f};
    soc_kf_init();
    uint32_t t = 0u, last = 0u;
    int gated_under_load = 0;

    for (; t < 10000u; t += 20u) {
        const float i_cell = 60.0f / (float)SOC_KF_NP;
        cell_step(&cell, i_cell, 0.020f);
        const float v_cell = cell_terminal(&cell, i_cell);
        soc_kf_feed_bms(600, (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 180, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            if (s.dbg.flags & SOC_KF_FLAG_GATED)
                gated_under_load = 1;
            last = t;
        }
    }
    check(s.init_method == 2, "seeded from Orion, under load");
    printf("    after 10 s under load: %.1f%% (truth %.1f%%), gated=%d\n", s.dbg.soc * 100.0f,
           cell.z * 100.0f, gated_under_load);
    check(gated_under_load, "a 60-point seed error is gated");

    for (; t < 130000u; t += 20u) {
        const float v_cell = cell_terminal(&cell, 0.0f);
        soc_kf_feed_bms(0, (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 180, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            last = t;
        }
    }
    printf("    after 120 s at rest: %.2f%% (truth %.2f%%), v1 %.4f V\n", s.dbg.soc * 100.0f,
           cell.z * 100.0f, s.v1);
    check_near(s.dbg.soc, cell.z, 0.02f, "rest reopens the gate and the filter recovers");
}

static void test_temperature_compensation(void)
{
    printf("test_temperature_compensation\n");
    const uint8_t temps[] = {15, 45};
    for (size_t i = 0; i < sizeof(temps); i++) {
        const uint8_t tc = temps[i];
        const float r_scale = temp_scale(tc);
        cell_t cell = {0.60f, 0.0f};
        soc_kf_init();
        uint32_t t = 0u, last = 0u;
        while (t < 60000u) {
            const float i_pack = (t < 2500u) ? 0.0f : 100.0f;
            const float i_cell = i_pack / (float)SOC_KF_NP;
            const float r1 = lut(soc_kf_r1, cell.z, 0) * r_scale;
            const float a = expf(-0.020f / SOC_KF_TAU1_S);
            cell.z -= i_cell * 0.020f / (3600.0f * SOC_KF_CAP_AH);
            cell.v1 = a * cell.v1 + r1 * (1.0f - a) * i_cell;
            const float v_cell =
                lut(soc_kf_ocv, cell.z, 0) - i_cell * lut(soc_kf_r0, cell.z, 0) * r_scale - cell.v1;
            soc_kf_feed_bms((int16_t)lrintf(i_pack * 10.0f),
                            (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), tc, 120, t);
            if (t - last >= 90u) {
                soc_kf_update(t);
                last = t;
            }
            t += 20u;
        }
        const float err = fabsf(s.dbg.soc - cell.z);
        printf("    %u C -> err %.2f%% SoC\n", tc, err * 100.0f);
        check(err < 0.02f, "temp compensation keeps error under 2% SoC");
    }
}

#define VBAT_GOOD 3600u /* 3.6 V/cell */
#define VBAT_BAD  1500u /* 1.5 V/cell, below SOC_KF_VCELL_MIN */

/* Frames every 20 ms until end, updating on each 100 ms boundary */
static void feed_until(uint32_t *t, uint32_t end, int16_t ibat, uint16_t vbat, uint8_t soc_raw)
{
    for (; *t < end; *t += 20u) {
        soc_kf_feed_bms(ibat, vbat, 25, soc_raw, *t);
        if (*t % 100u == 0u)
            soc_kf_update(*t);
    }
}

static void test_temp_scale_limits(void)
{
    printf("test_temp_scale_limits\n");
    check_near(temp_scale(0), soc_kf_r_temp[0], 1e-6f, "below 10 C holds the coldest scale");
    check_near(temp_scale(80), soc_kf_r_temp[SOC_KF_TEMP_N_BP - 1], 1e-6f,
               "above 55 C holds the hottest scale");
    check_near(temp_scale(25), 1.0f, 1e-6f, "25 C is the reference");
}

static void test_quiet_before_seed(void)
{
    printf("test_quiet_before_seed\n");
    soc_kf_init();
    soc_kf_update(1000u);
    const soc_kf_debug_t *kf = soc_kf_get_debug();
    check(kf == &s.dbg, "debug view is the live state");
    check(kf->flags == SOC_KF_FLAG_FROZEN, "no BMS yet: frozen and unseeded");
    check(!soc_kf_valid(kf), "not valid before any BMS frame");

    /* BMS seen once, then lost before the filter could seed */
    soc_kf_feed_bms(0, VBAT_GOOD, 25, 120, 2000u);
    soc_kf_update(2000u + SOC_KF_STALE_MS + 1u);
    check(kf->flags == SOC_KF_FLAG_FROZEN, "stale before seeding stays unseeded");
    check(!s.initialised, "no seed from a single stale frame");
}

static void test_blackout_after_orion_seed(void)
{
    printf("test_blackout_after_orion_seed\n");
    uint32_t t = 0u;
    soc_kf_init();

    /* Bad voltage blocks OCV and measurements, leaving pure coulomb counting */
    feed_until(&t, SOC_KF_INIT_TIMEOUT_MS + 200u, 1000, VBAT_BAD, 0);
    check(s.init_method == 2, "Orion seeds even when the voltage is implausible");
    check(s.pending_ms > 0u, "frames since the last update are pending");

    const uint32_t quiet = t + SOC_KF_STALE_MS;
    soc_kf_update(quiet);
    check((s.dbg.flags & SOC_KF_FLAG_FROZEN) != 0, "frozen once the BMS goes quiet");
    check((s.dbg.flags & SOC_KF_FLAG_OCV_INIT) == 0, "Orion seed not reported as OCV");
    check((s.dbg.flags & SOC_KF_FLAG_CLAMPED) != 0, "pending discharge clamps at empty");
    check_near(s.dbg.soc, 0.0f, 1e-6f, "held at empty");

    const float p00 = s.p00;
    soc_kf_update(quiet);
    check(s.p00 == p00, "no time passed, no blackout growth");

    /* Days of silence saturate the covariance rather than run away */
    soc_kf_update(quiet + 100000000u);
    check(s.p00 == SOC_KF_P_MAX, "SoC variance capped");
    check(s.p11 == SOC_KF_P_MAX, "RC variance capped");
}

static void test_full_pack_clamps(void)
{
    printf("test_full_pack_clamps\n");
    uint32_t t = 0u;
    soc_kf_init();

    /* Orion past 100% while charging */
    feed_until(&t, SOC_KF_INIT_TIMEOUT_MS + 200u, -1000, VBAT_BAD, 255);
    check(s.init_method == 2, "seeded from Orion");
    check_near(s.dbg.soc, 1.0f, 1e-6f, "Orion above 100% seeds at full");
    check((s.dbg.flags & SOC_KF_FLAG_CLAMPED) != 0, "charge past full clamps");
}

static void test_feed_gaps(void)
{
    printf("test_feed_gaps\n");
    soc_kf_init();
    soc_kf_feed_bms(0, VBAT_GOOD, 25, 120, 0u);
    soc_kf_feed_bms(0, VBAT_GOOD, 25, 120, 20u);
    check(s.rest_ms == 20u, "rest accumulates between frames");
    check(s.pending_ms == 20u, "charge accumulates between frames");

    const uint32_t late = 20u + SOC_KF_STALE_MS + 1u;
    soc_kf_feed_bms(0, VBAT_GOOD, 25, 120, late);
    check(s.rest_ms == 0u, "a frame gap restarts the rest timer");
    check(s.pending_ms == 20u, "the gap itself is not integrated");

    s.rest_ms = 0xFFFF0000u;
    soc_kf_feed_bms(0, VBAT_GOOD, 25, 120, late + 20u);
    check(s.rest_ms == 0xFFFF0000u, "rest timer saturates instead of wrapping");
}

static void test_update_timing(void)
{
    printf("test_update_timing\n");
    uint32_t t = 0u;
    soc_kf_init();
    feed_until(&t, SOC_KF_INIT_TIMEOUT_MS + 200u, 400, VBAT_BAD, 120);
    check(s.initialised, "seeded");

    /* 2 s of frames with no update, at 40 A pack (10 A cell) */
    soc_kf_update(t);
    s.v1 = 0.1f;
    const uint32_t end = t + 2000u;
    for (t += 20u; t <= end; t += 20u) {
        soc_kf_feed_bms(400, VBAT_BAD, 25, 120, t);
    }
    t -= 20u;
    soc_kf_update(t);
    float a = expf(-0.5f / SOC_KF_TAU1_S);
    float want = a * 0.1f + lut(soc_kf_r1, s.z, 0) * (1.0f - a) * 10.0f;
    check_near(s.v1, want, 1e-5f, "a long gap between updates counts as 0.5 s");

    /* No frame since the last update: falls back to the latest current */
    const float v1 = s.v1;
    soc_kf_update(t + 50u);
    a = expf(-0.05f / SOC_KF_TAU1_S);
    want = a * v1 + lut(soc_kf_r1, s.z, 0) * (1.0f - a) * 10.0f;
    check_near(s.v1, want, 1e-5f, "no pending charge uses the instantaneous current");
}

static void test_covariance_guards(void)
{
    printf("test_covariance_guards\n");
    run_sim(0.60f, 60.0f, 0.0f, 10.0f, 1, NULL);
    const uint16_t vbat = (uint16_t)(s.vbat_raw);
    uint32_t t = 10000u;

    /* Variances that went negative skip the measurement and get floored */
    const float z = s.z;
    s.p00 = -1e-3f;
    s.p11 = -1e-3f;
    s.p01 = 0.0f;
    s.p10 = 0.0f;
    soc_kf_feed_bms(0, vbat, 25, 120, t);
    soc_kf_update(t);
    check(s.p00 == SOC_KF_P_FLOOR, "SoC variance floored");
    check(s.p11 == SOC_KF_P_FLOOR, "RC variance floored");
    check_near(s.z, z, 1e-6f, "no measurement step without a positive innovation variance");

    /* Correlation past +-sqrt(p00 p11) is pulled back to the boundary */
    s.p00 = 1e-4f;
    s.p11 = 1e-4f;
    s.p01 = 1.0f;
    s.p10 = 1.0f;
    t += 20u;
    soc_kf_feed_bms(0, VBAT_BAD, 25, 120, t);
    soc_kf_update(t);
    check_near(s.p01, sqrtf(s.p00 * s.p11), 1e-9f, "positive correlation clamped");
    check(s.p01 == s.p10, "kept symmetric");

    s.p01 = -1.0f;
    s.p10 = -1.0f;
    t += 20u;
    soc_kf_feed_bms(0, VBAT_BAD, 25, 120, t);
    soc_kf_update(t);
    check_near(s.p01, -sqrtf(s.p00 * s.p11), 1e-9f, "negative correlation clamped");
    check(s.p01 == s.p10, "kept symmetric");
}

static void test_pack_saturates(void)
{
    printf("test_pack_saturates\n");
    uint8_t d[8];
    soc_kf_init();

    s.dbg.soc = 7.0f;
    s.dbg.innovation = 10.0f;
    s.dbg.charge_ah = 1000.0f;
    soc_kf_pack_state(d);
    check((d[0] | d[1] << 8) == 65535, "SoC saturates high");
    check((int16_t)(uint16_t)(d[2] | d[3] << 8) == 32767, "innovation saturates high");
    check((int16_t)(uint16_t)(d[4] | d[5] << 8) == 32767, "charge saturates high");

    s.dbg.soc = -0.5f;
    s.dbg.innovation = -10.0f;
    s.dbg.charge_ah = -1000.0f;
    soc_kf_pack_state(d);
    check((d[0] | d[1] << 8) == 0, "SoC saturates low");
    check((int16_t)(uint16_t)(d[2] | d[3] << 8) == -32768, "innovation saturates low");
    check((int16_t)(uint16_t)(d[4] | d[5] << 8) == -32768, "charge saturates low");
}

int main(void)
{
    printf("=== soc_kf host tests ===\n\n");
    test_lut_breakpoints();
    test_ocv_inverse();
    test_ocv_init_at_rest();
    test_converges_from_bad_seed();
    test_discharge_lowers_soc();
    test_coulomb_accuracy();
    test_staleness_freezes();
    test_packing();
    test_clamping();
    test_rejects_bad_voltage();
    test_orion_seed_scaling();
    test_in_range_garbage_is_gated();
    test_gated_filter_recovers_at_rest();
    test_temperature_compensation();
    test_temp_scale_limits();
    test_quiet_before_seed();
    test_blackout_after_orion_seed();
    test_full_pack_clamps();
    test_feed_gaps();
    test_update_timing();
    test_covariance_guards();
    test_pack_saturates();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
