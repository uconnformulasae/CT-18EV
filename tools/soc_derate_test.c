/* Host tests for soc_derate */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../Core/Src/soc_derate.c"
#include "../Core/Src/soc_kf.c"

#define KF_OK   (SOC_KF_FLAG_INIT | SOC_KF_FLAG_BMS_LIVE)
#define STEP_MS 5u
#define BMS_DCL 200u

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

static void check_eq(long got, long want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL: %s (got %ld want %ld)\n", what, got, want);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (fabsf(got - want) > tol) {
        failures++;
        printf("  FAIL: %s (got %g want %g)\n", what, got, want);
    }
}

static soc_kf_debug_t kf;

static const soc_kf_debug_t *kf_at(float soc, uint8_t flags)
{
    kf.soc = soc;
    kf.flags = flags;
    return &kf;
}

/* Independent of interp() so the test doesn't check the code against itself */
static uint32_t expected_dcl(float soc)
{
    const soc_derate_pt_t *p = tune->soc.pts;
    const int n = tune->soc.n;
    float a = p[n - 1].amps;

    if (soc <= p[0].x) {
        a = p[0].amps;
    } else {
        for (int i = 0; i < n - 1; i++) {
            if (soc <= p[i + 1].x) {
                const float f = (soc - p[i].x) / (p[i + 1].x - p[i].x);
                a = p[i].amps + f * (p[i + 1].amps - p[i].amps);
                break;
            }
        }
    }
    return (a > 0.0f) ? (uint32_t)lroundf(a) : 0u;
}

/* 0.1 mV raw, as Orion sends it */
static uint16_t raw(float v)
{
    return (uint16_t)lroundf(v * 10000.0f);
}

/* ---- every tune in soc_derate_tunes.h ---- */

static int tune_count;
static uint8_t tune_ids[32];

static void check_tune(const char *name, soc_derate_tune_t t)
{
    char what[96];

    snprintf(what, sizeof(what), "%s: has points", name);
    check(t.soc.n >= 1, what);

    for (int i = 0; i < t.soc.n; i++) {
        snprintf(what, sizeof(what), "%s: point %d SoC within 0-1", name, i);
        check(t.soc.pts[i].x >= 0.0f && t.soc.pts[i].x <= 1.0f, what);
        snprintf(what, sizeof(what), "%s: point %d amps not negative", name, i);
        check(t.soc.pts[i].amps >= 0.0f, what);
        if (i > 0) {
            snprintf(what, sizeof(what), "%s: points sorted by SoC (%d)", name, i);
            check(t.soc.pts[i].x > t.soc.pts[i - 1].x, what);
        }
    }

    for (int i = 0; i < tune_count; i++) {
        snprintf(what, sizeof(what), "%s: id %u unique", name, t.id);
        check(tune_ids[i] != t.id, what);
    }
    if (tune_count < (int)sizeof(tune_ids)) {
        tune_ids[tune_count++] = t.id;
    }
}

static void test_tunes(void)
{
    printf("test_tunes\n");
    tune_count = 0;

/* Re-read the tunes file, turning each definition into a check */
#undef SOC_DERATE_TUNES_H
#undef SOC_DERATE_TUNE_DEF
#define SOC_DERATE_TUNE_DEF(name, ...) check_tune(#name, (soc_derate_tune_t){__VA_ARGS__})
#include "soc_derate_tunes.h"

    check(tune_count >= 1, "found tunes");
    printf("  %d tunes checked\n", tune_count);
}

/* ---- the active tune's map ---- */

static void test_map(void)
{
    printf("test_map\n");
    soc_derate_init();

    int bad = 0;
    for (int i = 0; i <= 100; i++) {
        const float soc = (float)i * 0.01f;
        const uint32_t got = soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
        if (got != expected_dcl(soc)) {
            if (bad++ < 3) {
                printf("  soc %.2f: got %u want %u\n", soc, got, expected_dcl(soc));
            }
        }
    }
    check_eq(bad, 0, "map matches tune at every SoC %");

    const soc_derate_pt_t *p = tune->soc.pts;
    const int n = tune->soc.n;
    for (int i = 0; i < n; i++) {
        check_eq(soc_derate_dcl(kf_at(p[i].x, KF_OK), BMS_DCL, 0u, STEP_MS),
                 lroundf(p[i].amps), "exact at each point");
    }
    check_eq(soc_derate_dcl(kf_at(p[0].x - 0.05f, KF_OK), BMS_DCL, 0u, STEP_MS),
             lroundf(p[0].amps), "held below first point");
    check_eq(soc_derate_dcl(kf_at(p[n - 1].x + 0.05f, KF_OK), BMS_DCL, 0u, STEP_MS),
             lroundf(p[n - 1].amps), "held above last point");
    check_eq(soc_derate_get_debug()->src, SOC_DERATE_SRC_SOC, "source is SoC map");
}

/* ---- weakest cell gap ---- */

static void test_gap(void)
{
    printf("test_gap\n");
    const float soc = 0.80f;
    const float avg_v = 3.700f, low_v = 3.620f;
    const float want_gap = soc_kf_ocv_to_soc(avg_v) - soc_kf_ocv_to_soc(low_v);
    char what[64];

    soc_derate_init();
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    check_near(soc_derate_get_debug()->gap, 0.0f, 1e-6f, "no gap before 0x79F");
    check_eq(soc_derate_get_debug()->cell_live, 0, "cell data not live yet");

    soc_derate_feed_cells(raw(avg_v), raw(avg_v), 0u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    check_near(soc_derate_get_debug()->gap, 0.0f, 1e-6f, "balanced pack has no gap");
    check_eq(soc_derate_get_debug()->cell_live, 1, "cell data live");

    soc_derate_init();
    soc_derate_feed_cells(raw(low_v), raw(avg_v), 0u);
    const uint32_t dcl = soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    check(want_gap > 0.02f, "test cell is noticeably low");
    check_near(soc_derate_get_debug()->gap, want_gap, 1e-4f, "first reading applies at once");
    check_near(soc_derate_get_debug()->weak_soc, soc - want_gap, 1e-4f, "weak cell soc");
    check_eq(dcl, expected_dcl(soc - want_gap), "map runs on weak cell soc");

    /* Pack rebalances: gap decays with the filter time constant */
    uint32_t t = 0;
    const int steps = (int)(SOC_DERATE_GAP_TAU_S * 1000.0f / STEP_MS);
    for (int i = 0; i < steps; i++) {
        t += STEP_MS;
        soc_derate_feed_cells(raw(avg_v), raw(avg_v), t);
        soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, t, STEP_MS);
    }
    snprintf(what, sizeof(what), "gap at 1/e after %.0f s", SOC_DERATE_GAP_TAU_S);
    check_near(soc_derate_get_debug()->gap, want_gap * expf(-1.0f), want_gap * 0.01f, what);

    /* One bad sample barely moves it */
    soc_derate_init();
    soc_derate_feed_cells(raw(avg_v), raw(avg_v), 0u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    soc_derate_feed_cells(raw(3.0f), raw(avg_v), STEP_MS);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, STEP_MS, STEP_MS);
    check(soc_derate_get_debug()->gap < 0.001f, "single sag sample filtered");

    /* Low above average is nonsense, never a negative gap */
    soc_derate_init();
    soc_derate_feed_cells(raw(3.75f), raw(3.70f), 0u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    check_near(soc_derate_get_debug()->gap, 0.0f, 1e-6f, "gap never negative");

    /* A stall longer than the time constant takes the new gap outright */
    soc_derate_init();
    soc_derate_feed_cells(raw(avg_v), raw(avg_v), 0u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    soc_derate_feed_cells(raw(low_v), raw(avg_v), 0u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, (uint32_t)(SOC_DERATE_GAP_TAU_S * 2000.0f));
    check_near(soc_derate_get_debug()->gap, want_gap, 1e-4f, "filter weight capped at 1");
}

static void test_stale(void)
{
    printf("test_stale\n");
    const float soc = 0.80f;

    soc_derate_init();
    soc_derate_feed_cells(raw(3.62f), raw(3.70f), 1000u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 1000u, STEP_MS);
    const float gap = soc_derate_get_debug()->gap;

    const uint32_t edge = 1000u + SOC_DERATE_CELL_STALE_MS;
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, edge, STEP_MS);
    check_eq(soc_derate_get_debug()->cell_live, 1, "live at the stale limit");

    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, edge + 1u, STEP_MS);
    check_eq(soc_derate_get_debug()->cell_live, 0, "stale past the limit");
    check_near(soc_derate_get_debug()->gap, gap, 1e-6f, "gap held while stale");

    for (uint32_t t = edge; t < edge + 120000u; t += 100u) {
        soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, t, 100u);
    }
    check_near(soc_derate_get_debug()->gap, gap, 1e-6f, "gap held for minutes");

    /* Tick wrap */
    soc_derate_init();
    soc_derate_feed_cells(raw(3.70f), raw(3.70f), 0xFFFFFF00u);
    soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0x00000010u, STEP_MS);
    check_eq(soc_derate_get_debug()->cell_live, 1, "live across tick wrap");
}

static void test_fallback(void)
{
    printf("test_fallback\n");
    soc_derate_init();

    check_eq(soc_derate_dcl(kf_at(0.5f, 0), BMS_DCL, 0u, STEP_MS), BMS_DCL, "kf not init");
    check_eq(soc_derate_get_debug()->src, SOC_DERATE_SRC_BMS, "source is BMS");
    check_eq(soc_derate_dcl(kf_at(0.5f, SOC_KF_FLAG_INIT), BMS_DCL, 0u, STEP_MS), BMS_DCL,
             "bms not live");
    check_eq(soc_derate_dcl(kf_at(0.5f, KF_OK | SOC_KF_FLAG_VBAD), BMS_DCL, 0u, STEP_MS),
             BMS_DCL, "bad voltage");
    check_eq(soc_derate_dcl(kf_at(0.5f, KF_OK | SOC_KF_FLAG_VBAD), 0u, 0u, STEP_MS), 0,
             "bms dcl of 0 passes through");
    check_eq(soc_derate_dcl(kf_at(0.5f, KF_OK), BMS_DCL, 0u, STEP_MS), expected_dcl(0.5f),
             "back to map when kf valid");

    check(expected_dcl(0.9f) > 0u, "map allows current at 90%");
    check_eq(soc_derate_dcl(kf_at(0.9f, KF_OK), 0u, 0u, STEP_MS), 0, "bms dcl of 0 beats map");
    check_eq(soc_derate_get_debug()->src, SOC_DERATE_SRC_BMS, "source is BMS at 0 A");
    check_eq(soc_derate_dcl(kf_at(0.9f, KF_OK), 1u, 0u, STEP_MS), expected_dcl(0.9f),
             "any nonzero bms dcl is ignored");
    check_eq(soc_derate_get_debug()->src, SOC_DERATE_SRC_SOC, "source back to SoC");
}

static void test_pack(void)
{
    printf("test_pack\n");
    const float soc = 0.80f;

    soc_derate_init();
    soc_derate_feed_cells(raw(3.62f), raw(3.70f), 0u);
    const uint32_t dcl = soc_derate_dcl(kf_at(soc, KF_OK), BMS_DCL, 0u, STEP_MS);
    const soc_derate_debug_t *d = soc_derate_get_debug();

    uint8_t b[8];
    memset(b, 0xAA, sizeof(b));
    soc_derate_pack_debug(b);
    check_eq(b[0], tune->id, "tune id");
    check_eq(b[1] & 0x01, SOC_DERATE_SRC_SOC, "source bit");
    check_eq((b[1] >> 1) & 0x01, 1, "cell live bit");
    check_eq(b[1] & 0xFC, 0, "unused bits clear");
    check_eq(b[2] | b[3] << 8, (long)dcl, "dcl");
    check_eq(b[4] | b[5] << 8, lroundf(d->weak_soc * 10000.0f), "weak soc 0.01%");
    check_eq(b[6] | b[7] << 8, lroundf(d->gap * 10000.0f), "gap 0.01%");

    soc_derate_dcl(kf_at(0.5f, 0), 300u, 0u, STEP_MS);
    soc_derate_pack_debug(b);
    check_eq(b[1] & 0x01, SOC_DERATE_SRC_BMS, "fallback source bit");
    check_eq(b[2] | b[3] << 8, 300, "dcl above 255");

    soc_derate_init();
    soc_derate_dcl(kf_at(0.0f, KF_OK), BMS_DCL, 0u, STEP_MS);
    soc_derate_feed_cells(raw(3.0f), raw(4.1f), 0u);
    soc_derate_dcl(kf_at(0.0f, KF_OK), BMS_DCL, 0u, STEP_MS);
    soc_derate_pack_debug(b);
    check_eq(b[4] | b[5] << 8, 0, "negative weak soc clamps to 0");

    soc_derate_init();
    soc_derate_dcl(kf_at(7.0f, KF_OK), BMS_DCL, 0u, STEP_MS);
    soc_derate_pack_debug(b);
    check_eq(b[4] | b[5] << 8, 65535, "out of range weak soc saturates high");
}

int main(void)
{
    test_tunes();
    test_map();
    test_gap();
    test_stale();
    test_fallback();
    test_pack();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
