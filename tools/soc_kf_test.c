/* Host tests for soc_kf. Includes the .c directly to reach the static helpers.
 *   cc -DSOC_KF_HOST -I../Core/Inc -o soc_kf_test soc_kf_test.c -lm
 */

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

/* Returns final |error| in SoC. */
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

    /* Flush the coulombs banked since the last update - those are measured and
     * the filter is right to keep them. What must not move the estimate is the
     * silence that follows. */
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
    /* 15% of 4.99 Ah at 25 A/cell drains in ~108 s: 200 s must hit the floor. */
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

    /* Garbage frame: 6500 V pack = 65 V/cell. The filter must not chase it. */
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

static void test_rest_timer_uses_wall_time(void)
{
    printf("test_rest_timer_uses_wall_time\n");
    cell_t cell = {0.62f, 0.0f};
    soc_kf_init();

    uint32_t t = 0u, last = 0u;
    while (t < SOC_KF_INIT_TIMEOUT_MS - 100u) {
        const float v_cell = cell_terminal(&cell, 0.0f);
        soc_kf_feed_bms(0, (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 124, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            last = t;
        }
        t += 50u;
    }
    check(s.init_method == 1, "rest-OCV seed still lands at 20 Hz");
    check_near(s.z, 0.62f, 0.02f, "seeded from the rest OCV");
}

static void test_dropout_breaks_rest_streak(void)
{
    printf("test_dropout_breaks_rest_streak\n");
    soc_kf_init();

    uint32_t t = 0u;
    for (; t < 1900u; t += 20u)
        soc_kf_feed_bms(0, 3800u, 25, 124, t);
    printf("    rest_ms before the gap = %u\n", (unsigned)s.rest_ms);
    check(s.rest_ms >= 1800u && s.rest_ms < 2000u, "rest accrues in wall time");

    t += 10000u;
    soc_kf_feed_bms(0, 3800u, 25, 124, t);
    check(s.rest_ms == 0u, "a dropout clears the rest streak");
}

static void test_freeze_keeps_measured_charge(void)
{
    printf("test_freeze_keeps_measured_charge\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 0.0f, 6.0f, 1, &est);
    check(s.initialised, "seeded before the dropout");

    const float z0 = s.z;
    const float ah0 = s.charge_ah;

    uint32_t t = 6000u;
    for (int k = 0; k < 25; k++) {
        soc_kf_feed_bms(2000, 3600u, 25, 120, t);
        t += 20u;
    }
    soc_kf_update(t + SOC_KF_STALE_MS + 100u);

    check((s.dbg.flags & SOC_KF_FLAG_FROZEN) != 0, "FROZEN on the stale update");
    const float dz = z0 - s.z;
    const float dah = s.charge_ah - ah0;
    printf("    banked %.4f Ah, z fell %.6f\n", dah, dz);
    check(dah > 0.02f, "charge was banked");
    check_near(dz, dah / (float)SOC_KF_NP / SOC_KF_CAP_AH, 1e-4f, "z absorbs the pre-gap coulombs");
}

static void test_frozen_holds_stale_inputs(void)
{
    printf("test_frozen_holds_stale_inputs\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 50.0f, 20.0f, 1, &est);
    const float i_live = s.dbg.i_pack;
    const float v_live = s.dbg.v_pack;
    check((s.dbg.flags & SOC_KF_FLAG_OCV_INIT) != 0, "seeded from rest OCV");

    soc_kf_feed_bms(3000, 4000u, 55, 200, 20000u);
    soc_kf_update(20000u + SOC_KF_STALE_MS + 100u);

    check((s.dbg.flags & SOC_KF_FLAG_FROZEN) != 0, "FROZEN");
    check((s.dbg.flags & SOC_KF_FLAG_OCV_INIT) != 0, "seed method survives the freeze");
    check_near(s.dbg.i_pack, i_live, 1e-6f, "stale current not republished as fresh");
    check_near(s.dbg.v_pack, v_live, 1e-6f, "stale voltage not republished as fresh");
}

static void test_v1_uses_window_mean_current(void)
{
    printf("test_v1_uses_window_mean_current\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 0.0f, 6.0f, 1, &est);

    uint32_t t = 6000u;
    soc_kf_feed_bms(0, 65000u, 25, 120, t);
    soc_kf_update(t);
    t += 20u;
    for (int k = 0; k < 19; k++) {
        soc_kf_feed_bms(0, 65000u, 25, 120, t);
        t += 20u;
    }
    soc_kf_feed_bms(3000, 65000u, 25, 120, t);
    soc_kf_update(t);

    check((s.dbg.flags & SOC_KF_FLAG_VBAD) != 0, "correction skipped");

    const float a = expf(-0.400f / SOC_KF_TAU1_S);
    const float expect = lut(soc_kf_r1, s.z, 0) * (1.0f - a) * 3.75f;
    printf("    v1 = %.6f V, window-mean prediction %.6f V\n", s.v1, expect);
    check_near(s.v1, expect, 2e-5f, "v1 driven by the window mean, not the last sample");
}

static void test_covariance_stays_psd(void)
{
    printf("test_covariance_stays_psd\n");
    float est = 0.0f;
    run_sim(0.72f, 50.0f, 40.0f, 120.0f, 0, &est);
    check(s.p00 > 0.0f && s.p11 > 0.0f, "diagonal stays positive");
    check(s.p01 == s.p10, "P symmetric");
    printf("    p00=%.3e p01=%.3e p11=%.3e\n", s.p00, s.p01, s.p11);
    check(s.p01 * s.p01 <= s.p00 * s.p11 * 1.0001f, "P positive semi-definite");
}

static void seed_at_rest(float true_z, uint32_t rest_hold_ms)
{
    cell_t cell = {true_z, 0.0f};
    soc_kf_init();
    const float v_cell = cell_terminal(&cell, 0.0f);
    const uint16_t vbat = (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f);

    uint32_t t = 0u;
    for (; t < rest_hold_ms; t += 20u)
        soc_kf_feed_bms(0, vbat, 25, 110, t);

    uint32_t last = t;
    while (!s.initialised && t < rest_hold_ms + 10000u) {
        soc_kf_feed_bms(0, vbat, 25, 110, t);
        if (t - last >= 90u) {
            soc_kf_update(t);
            last = t;
        }
        t += 20u;
    }
}

static void test_short_rest_seed_is_not_overconfident(void)
{
    printf("test_short_rest_seed_is_not_overconfident\n");
    seed_at_rest(0.72f, 0u);

    check(s.init_method == 1, "rest-OCV seed");
    printf("    p00 = %.3e (sigma %.1f%% SoC), p01 = %.3e, p11 = %.3e\n", s.p00,
           sqrtf(s.p00) * 100.0f, s.p01, s.p11);

    check(sqrtf(s.p00) > 0.05f, "a 2 s rest does not claim 1% SoC confidence");
    check(s.p01 > 0.0f, "z and V1 seed errors are correlated, not independent");
    check(s.p01 * s.p01 <= s.p00 * s.p11 * 1.0001f, "seed covariance is PSD");
}

static void test_long_rest_seed_is_confident(void)
{
    printf("test_long_rest_seed_is_confident\n");
    seed_at_rest(0.55f, 90000u);

    check(s.init_method == 1, "rest-OCV seed");
    printf("    p00 = %.3e (sigma %.2f%% SoC)\n", s.p00, sqrtf(s.p00) * 100.0f);
    check(sqrtf(s.p00) < 0.02f, "a relaxed pack still seeds tight");
    check_near(s.z, 0.55f, 0.01f, "and seeds accurately");
}

static float p00_growth_over_10s(uint32_t update_period_ms)
{
    seed_at_rest(0.55f, 90000u);
    uint32_t t = 91000u;
    const float p0 = s.p00;
    uint32_t last = t - 20u;
    const uint32_t end = t + 10000u;
    for (; t < end; t += 20u) {
        soc_kf_feed_bms(0, 65000u, 25, 120, t);
        if (t - last >= update_period_ms) {
            soc_kf_update(t);
            last = t;
        }
    }
    return s.p00 - p0;
}

static void test_process_noise_scales_with_dt(void)
{
    printf("test_process_noise_scales_with_dt\n");
    const float slow = p00_growth_over_10s(200u);
    const float fast = p00_growth_over_10s(40u);
    printf("    10 s at  5 Hz -> dp00 = %.4e\n", slow);
    printf("    10 s at 25 Hz -> dp00 = %.4e\n", fast);
    check(fabsf(fast - slow) < 0.05f * slow, "Q is a rate, not a per-call constant");
}

static void test_freeze_inflates_covariance(void)
{
    printf("test_freeze_inflates_covariance\n");
    float est = 0.0f;
    run_sim(0.60f, 60.0f, 50.0f, 20.0f, 1, &est);
    soc_kf_update(19980u);
    const float p_before = s.p00;

    soc_kf_update(19980u + 10000u);
    check((s.dbg.flags & SOC_KF_FLAG_FROZEN) != 0, "FROZEN");
    printf("    sigma %.2f%% -> %.2f%% SoC across a 10 s blackout\n", sqrtf(p_before) * 100.0f,
           sqrtf(s.p00) * 100.0f);
    check(s.p00 > p_before, "a blackout costs confidence");
    check(sqrtf(s.p00) > 0.01f, "10 s of silence is worth at least 1% SoC");
}

static float run_cold(float z0, float amps, float secs, float r_scale, int *gated)
{
    cell_t cell = {z0, 0.0f};
    soc_kf_init();
    uint32_t t = 0u, last = 0u;
    *gated = 0;
    while (t < (uint32_t)(secs * 1000.0f)) {
        const float i_pack = (t < 2500u) ? 0.0f : amps;
        const float i_cell = i_pack / (float)SOC_KF_NP;
        cell_step(&cell, i_cell, 0.020f);
        const float extra = i_cell * lut(soc_kf_r0, cell.z, 0) * (r_scale - 1.0f);
        const float v_cell = cell_terminal(&cell, i_cell) - extra;
        soc_kf_feed_bms((int16_t)lrintf(i_pack * 10.0f),
                        (uint16_t)lrintf(v_cell * (float)SOC_KF_NS * 10.0f), 25, 120, t);
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

static void test_healthy_operation_never_gates(void)
{
    printf("test_healthy_operation_never_gates\n");
    int gated = 0;
    run_cold(0.72f, 40.0f, 120.0f, 1.0f, &gated);
    check(!gated, "40 A never gates");
    run_cold(0.60f, 150.0f, 60.0f, 1.0f, &gated);
    check(!gated, "150 A never gates");
    run_cold(0.50f, -100.0f, 60.0f, 1.0f, &gated);
    check(!gated, "regen never gates");
    run_cold(0.15f, 100.0f, 200.0f, 1.0f, &gated);
    check(!gated, "deep discharge never gates");
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

static void test_plausible_ir_error_is_not_gated(void)
{
    printf("test_plausible_ir_error_is_not_gated\n");
    int gated = 0;
    run_cold(0.60f, 150.0f, 60.0f, 0.85f, &gated);
    check(!gated, "a warm pack (0.85x IR) is not gated");
    run_cold(0.60f, 150.0f, 60.0f, 1.30f, &gated);
    check(!gated, "aged cells (1.30x IR) are not gated");
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

static void test_temp_scale(void)
{
    printf("test_temp_scale\n");
    check_near(temp_scale(25), 1.00f, 1e-4f, "25 C is 1.0x");
    check_near(temp_scale(10), 1.48f, 1e-4f, "10 C is 1.48x");
    check_near(temp_scale(5), 1.48f, 1e-4f, "sub-10 C clamps");
    check_near(temp_scale(55), 0.55f, 1e-4f, "55 C is 0.55x");
    check_near(temp_scale(65), 0.55f, 1e-4f, "above 55 C clamps");
    check_near(temp_scale(20), 1.13f, 1e-4f, "20 C is 1.13x");
    check_near(temp_scale(40), 0.71f, 1e-4f, "40 C is 0.71x");
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
    test_rest_timer_uses_wall_time();
    test_dropout_breaks_rest_streak();
    test_freeze_keeps_measured_charge();
    test_frozen_holds_stale_inputs();
    test_v1_uses_window_mean_current();
    test_covariance_stays_psd();
    test_short_rest_seed_is_not_overconfident();
    test_long_rest_seed_is_confident();
    test_process_noise_scales_with_dt();
    test_freeze_inflates_covariance();
    test_healthy_operation_never_gates();
    test_in_range_garbage_is_gated();
    test_plausible_ir_error_is_not_gated();
    test_gated_filter_recovers_at_rest();
    test_temp_scale();
    test_temperature_compensation();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
