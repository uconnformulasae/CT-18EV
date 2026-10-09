/* Host tests for regen */

#include <stdio.h>
#include <stdlib.h>

#undef REGEN_ENABLE
#define REGEN_ENABLE 1

#include "../Core/Src/regen.c"

#define KF_OK   (SOC_KF_FLAG_INIT | SOC_KF_FLAG_BMS_LIVE)
#define STEP_MS 5u
#define RPM     1500u
#define FULL    (-437)
#define V_PACK  400.0f

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

static soc_kf_debug_t kf;

static const soc_kf_debug_t *bms(float soc, uint8_t flags, float v_pack)
{
    kf.soc = soc;
    kf.flags = flags;
    kf.v_pack = v_pack;
    return &kf;
}

static int32_t step(int32_t drive, uint32_t rpm, float tps)
{
    return regen_update(drive, rpm, tps, bms(0.5f, KF_OK, V_PACK), 0u, STEP_MS);
}

static int32_t settle_v(int32_t drive, uint32_t rpm, float tps, float soc, uint8_t flags,
                        float v_pack)
{
    int32_t t = 0;
    for (int i = 0; i < 50; i++) {
        t = regen_update(drive, rpm, tps, bms(soc, flags, v_pack), 0u, STEP_MS);
    }
    return t;
}

static int32_t settle(int32_t drive, uint32_t rpm, float tps, float soc, uint8_t flags)
{
    return settle_v(drive, rpm, tps, soc, flags, V_PACK);
}

/* Mirrors tmap_lut in main.c */
static int32_t drive_for(float tps)
{
    const float x = fmaxf(TMAP_DRIVE_START, fminf(tps, TMAP_DEADBAND_HIGH));
    return (int32_t)((x - TMAP_DRIVE_START) / (TMAP_DEADBAND_HIGH - TMAP_DRIVE_START) *
                     (float)TORQUE_LIMIT_NM_X10);
}

static int32_t pedal(float tps)
{
    return step(drive_for(tps), RPM, tps);
}

static float expected_pedal(float x)
{
    if (x <= TMAP_DEADBAND_LOW) {
        return 1.0f;
    }
    if (x >= TMAP_REGEN_END) {
        return 0.0f;
    }
    const float u = (TMAP_REGEN_END - x) / (TMAP_REGEN_END - TMAP_DEADBAND_LOW);
    return u * u * (3.0f - 2.0f * u);
}

static void test_equation(void)
{
    printf("test_equation\n");
    int eq_bad = 0, clamp_bad = 0;
    for (int xi = 0; xi <= 35; xi++) {
        const float x = (float)xi / 100.0f;
        const float p = expected_pedal(x);

        for (uint32_t rpm = REGEN_RAMP_RPM; rpm <= 4000; rpm += 50) {
            const float y = (float)rpm / 60.0f;
            const int32_t want =
                -(int32_t)(10.0f * p * (REGEN_PEAK_NM / 100.0f) * (2.0f * y - y * y / 100.0f));
            regen_init();
            if (abs(settle(0, rpm, x, 0.5f, KF_OK) - want) > 1) {
                eq_bad++;
            }
        }

        for (uint32_t rpm = 4400; rpm <= 6000; rpm += 50) {
            const float nm = 25.0f * 4.0f * V_PACK / ((float)rpm * 2.0f * 3.14159265f / 60.0f);
            const int32_t want = -(int32_t)(10.0f * p * nm);
            regen_init();
            if (abs(settle(0, rpm, x, 0.5f, KF_OK) - want) > 1) {
                clamp_bad++;
            }
        }
    }
    check_eq(eq_bad, 0, "z = -(pedal)(2y - y^2/100) below the current clamp");
    check_eq(clamp_bad, 0, "z = -(pedal)(100 A * V / w) above it");
}

static void test_pedal(void)
{
    printf("test_pedal\n");
    regen_init();
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK), FULL, "full regen foot off");
    check_eq(settle(0, RPM, TMAP_DEADBAND_LOW, 0.5f, KF_OK), FULL, "held below low deadband");

    const float mid = (TMAP_DEADBAND_LOW + TMAP_REGEN_END) / 2.0f;
    const int32_t half = settle(0, RPM, mid, 0.5f, KF_OK);
    check(half >= -219 && half <= -217, "half regen mid pedal");

    check_eq(settle(0, RPM, TMAP_REGEN_END, 0.5f, KF_OK), 0, "zero at regen end");
    check_eq(settle(0, RPM, 0.37f, 0.5f, KF_OK), 0, "zero in coast band");
    check_eq(settle(0, RPM, 1.1f, 0.5f, KF_OK), 0, "zero at full pedal with no drive");
    check_eq(settle(0, RPM, -0.1f, 0.5f, KF_OK), FULL, "negative tps clamps to full regen");
}

static void test_lift_into_regen(void)
{
    printf("test_lift_into_regen\n");
    regen_init();
    int32_t last = settle(drive_for(0.6f), RPM, 0.6f, 0.5f, KF_OK);
    check(last > 0, "driving before lift");

    int early = 0, late = 0, rising = 0, jump = 0;
    for (int i = 120; i >= 0; i--) {
        const float tps = (float)i * 0.005f;
        const int32_t t = pedal(tps);
        if (tps >= TMAP_REGEN_END && t < 0) {
            early++;
        }
        if (tps < TMAP_REGEN_END - 0.01f && t >= 0) {
            late++;
        }
        if (t > last) {
            rising++;
        }
        if (abs(t - last) > jump) {
            jump = abs(t - last);
        }
        last = t;
    }
    check_eq(early, 0, "no regen above regen end");
    check_eq(late, 0, "regen once below regen end");
    check_eq(rising, 0, "torque only falls on lift");
    check(jump <= 20, "no torque step on lift");
    check_eq(last, FULL, "full regen foot off");
}

static void test_press_out_of_regen(void)
{
    printf("test_press_out_of_regen\n");
    regen_init();
    int32_t last = settle(0, RPM, 0.0f, 0.5f, KF_OK);

    int coast = 0, drive = 0, falling = 0, jump = 0;
    for (int i = 0; i <= 120; i++) {
        const float tps = (float)i * 0.005f;
        const int32_t t = pedal(tps);
        if (tps >= TMAP_REGEN_END && tps <= TMAP_DRIVE_START && t != 0) {
            coast++;
        }
        if (tps > TMAP_DRIVE_START + 0.01f && t <= 0) {
            drive++;
        }
        if (t < last) {
            falling++;
        }
        if (abs(t - last) > jump) {
            jump = abs(t - last);
        }
        last = t;
    }
    check_eq(coast, 0, "zero through the coast band");
    check_eq(drive, 0, "drive once past drive start");
    check_eq(falling, 0, "torque only rises on press");
    check(jump <= 20, "no torque step on press");
    check_eq(last, drive_for(120.0f * 0.005f), "full drive map after exit");
}

static void test_partial_lift(void)
{
    printf("test_partial_lift\n");
    regen_init();
    settle(drive_for(0.6f), RPM, 0.6f, 0.5f, KF_OK);
    check_eq(settle(0, RPM, 0.15f, 0.5f, KF_OK), -324, "partial regen at 15% pedal");
    check_eq(settle(0, RPM, 0.37f, 0.5f, KF_OK), 0, "coast at 37% pedal");
    check_eq(settle(drive_for(0.6f), RPM, 0.6f, 0.5f, KF_OK), drive_for(0.6f), "back to drive");
}

static void test_boundary_dither(void)
{
    printf("test_boundary_dither\n");
    int worst_regen = 0, worst_drive = 0;
    regen_init();
    for (int i = 0; i < 100; i++) {
        const int32_t t = pedal(TMAP_REGEN_END + ((i & 1) ? 0.002f : -0.002f));
        if (abs(t) > worst_regen) {
            worst_regen = abs(t);
        }
    }
    regen_init();
    for (int i = 0; i < 100; i++) {
        const int32_t t = pedal(TMAP_DRIVE_START + ((i & 1) ? 0.002f : -0.002f));
        if (abs(t) > worst_drive) {
            worst_drive = abs(t);
        }
    }
    check(worst_regen <= 5, "no chatter at regen end");
    check(worst_drive <= 10, "no chatter at drive start");
}

static void test_speed(void)
{
    printf("test_speed\n");
    regen_init();
    check_eq(settle(0, 0, 0.0f, 0.5f, KF_OK), 0, "none at standstill");
    check_eq(settle(0, REGEN_CUTOFF_RPM, 0.0f, 0.5f, KF_OK), 0, "none at cutoff");

    const int32_t a = settle(0, 520, 0.0f, 0.5f, KF_OK);
    const int32_t b = settle(0, 600, 0.0f, 0.5f, KF_OK);
    const int32_t c = settle(0, REGEN_RAMP_RPM, 0.0f, 0.5f, KF_OK);
    check(a < 0 && a > b && b > c, "ramps in above cutoff");

    check_eq(settle(0, 1000, 0.0f, 0.5f, KF_OK), -305, "equation at 1000 rpm");
    check_eq(settle(0, 2000, 0.0f, 0.5f, KF_OK), -555, "equation at 2000 rpm");
    check_eq(settle(0, 3000, 0.0f, 0.5f, KF_OK), -750, "equation at 3000 rpm");
    check_eq(settle(0, 6000, 0.0f, 0.5f, KF_OK), -636, "current clamp at max rpm");
    check_eq(settle(0, 12000, 0.0f, 0.5f, KF_OK), -318, "current clamp past max rpm");
}

static void test_charge_current(void)
{
    printf("test_charge_current\n");
    const float volts[] = {300.0f, 350.0f, 400.0f, 415.0f};
    float worst = 0.0f;
    for (int v = 0; v < 4; v++) {
        for (uint32_t rpm = REGEN_RAMP_RPM; rpm <= 9000; rpm += 50) {
            regen_init();
            const int32_t t = settle_v(0, rpm, 0.0f, 0.5f, KF_OK, volts[v]);
            const float amps = (float)-t / 10.0f * (float)rpm * RPM_TO_RAD_S / volts[v];
            if (amps > worst) {
                worst = amps;
            }
        }
    }
    check(worst <= 100.0f, "pack charge current within 25 A per cell");
    check(worst > 99.0f, "and uses the full limit");

    regen_init();
    check_eq(settle_v(0, 6000, 0.0f, 0.5f, KF_OK, 350.0f), -557, "clamp follows pack voltage");
    check_eq(settle_v(0, 3000, 0.0f, 0.5f, KF_OK, 0.0f), 0, "no regen without pack voltage");
}

static void test_never_positive(void)
{
    printf("test_never_positive\n");
    int bad = 0;
    for (int i = -10; i <= 110; i++) {
        for (uint32_t rpm = 0; rpm <= 15000; rpm += 125) {
            regen_init();
            const int32_t t = settle(0, rpm, (float)i / 100.0f, 0.5f, KF_OK);
            if (t > 0 || t < -(int32_t)(10.0f * REGEN_PEAK_NM)) {
                bad++;
            }
        }
    }
    check_eq(bad, 0, "regen stays within [-peak, 0]");
}

static void test_peak(void)
{
    printf("test_peak\n");
    regen_init();
    check_eq(settle_v(0, (uint32_t)REGEN_RPM_MAX, 0.0f, 0.5f, KF_OK, 2000.0f),
             -(int32_t)(10.0f * REGEN_PEAK_NM), "peak at max rpm without current clamp");
}

static void test_soc(void)
{
    printf("test_soc\n");
    regen_init();
    check_eq(settle(0, RPM, 0.0f, 0.97f, KF_OK), 0, "none above fade start");
    check_eq(settle(0, RPM, 0.0f, REGEN_SOC_OFF, KF_OK), 0, "none at fade start");
    check_eq(settle(0, RPM, 0.0f, 0.875f, KF_OK), -109, "quarter strength halfway through fade");
    check_eq(settle(0, RPM, 0.0f, 0.85f, KF_OK), -194, "curved fade at 85%");
    check_eq(settle(0, RPM, 0.0f, REGEN_SOC_FULL, KF_OK), FULL, "full at fade end");
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK), FULL, "full below fade end");
    check_eq(settle(0, RPM, 0.0f, 0.97f, KF_OK), 0, "follows soc back up");

    int32_t last = 0;
    int rising = 0;
    for (int i = 100; i >= 70; i--) {
        regen_init();
        const int32_t t = settle(0, RPM, 0.0f, (float)i / 100.0f, KF_OK);
        if (t > last) {
            rising++;
        }
        last = t;
    }
    check_eq(rising, 0, "regen only grows as soc falls");

    regen_init();
    check_eq(settle(0, RPM, 0.0f, 0.0f, SOC_KF_FLAG_BMS_LIVE), 0, "off before kf init");
    check_eq(settle(0, RPM, 0.0f, NAN, KF_OK), 0, "off on nan soc");

    regen_init();
    settle(0, RPM, 0.0f, 0.5f, KF_OK);
    check_eq(settle(0, RPM, 0.0f, 0.5f, SOC_KF_FLAG_INIT), 0, "off when bms stale");
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK), FULL, "back when bms returns");
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK | SOC_KF_FLAG_VBAD), 0, "off on bad voltage");
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK | SOC_KF_FLAG_GATED), FULL, "gated is fine");
}

static void test_cut(void)
{
    printf("test_cut\n");
    regen_init();
    settle(0, RPM, 0.0f, 0.5f, KF_OK);
    const soc_kf_debug_t *ok = bms(0.5f, KF_OK, V_PACK);
    check_eq(regen_update(0, RPM, 0.0f, ok, 1u, STEP_MS), 0, "cut drops regen at once");
    check_eq(regen_update(500, RPM, 0.9f, ok, 1u, STEP_MS), 500, "cut passes drive");
    check_eq(regen_update(0, RPM, 0.0f, ok, 1u, STEP_MS), 0, "cut from drive");
    check_eq(step(0, RPM, 0.0f), -100, "ramps back in");
    check_eq(settle(0, RPM, 0.0f, 0.5f, KF_OK), FULL, "settles");
}

static void test_slew(void)
{
    printf("test_slew\n");
    regen_init();
    check_eq(step(1000, RPM, 0.9f), 75, "tip-in from coast is limited");
    check_eq(step(1000, RPM, 0.9f), 150, "until past the band");
    check_eq(step(1000, RPM, 0.9f), 1000, "then free");
    check_eq(step(2000, RPM, 0.9f), 2000, "drive increase is free");
    check_eq(step(300, RPM, 0.9f), 300, "drive decrease is free");

    check_eq(step(0, RPM, 0.0f), 0, "lift-off drops to zero");
    int32_t last = 0;
    int bad = 0;
    for (int i = 0; i < 4; i++) {
        const int32_t t = step(0, RPM, 0.0f);
        if (t != last - REGEN_SLEW_DOWN * (int32_t)STEP_MS) {
            bad++;
        }
        last = t;
    }
    check_eq(bad, 0, "regen builds at the down rate");
    check_eq(step(0, RPM, 0.0f), FULL, "regen settles");

    last = FULL;
    bad = 0;
    for (int n = 0; last < REGEN_SLEW_BAND && n < 50; n++) {
        const int32_t t = step(1000, RPM, 0.9f);
        if (t - last != REGEN_SLEW_UP * (int32_t)STEP_MS) {
            bad++;
        }
        last = t;
    }
    check_eq(bad, 0, "regen releases at the up rate");
    check_eq(step(1000, RPM, 0.9f), 1000, "then free");

    regen_init();
    settle(0, RPM, 0.0f, 0.5f, KF_OK);
    check_eq(regen_update(1000, RPM, 0.9f, bms(0.5f, KF_OK, V_PACK), 0u, 1000u), FULL + 150,
             "long dt is clamped");
}

static void test_pack(void)
{
    printf("test_pack\n");
    regen_init();
    settle(0, RPM, 0.0f, 0.5f, KF_OK);

    uint8_t d[8] = {0};
    regen_pack_debug(d);
    check_eq(d[0], 0x0D, "flags: active, soc ok, soc scale");
    check_eq(d[1], 100, "pedal");
    check_eq(d[2], 43, "max regen (Nm)");
    check_eq((int16_t)(d[3] | d[4] << 8), FULL, "target");
    check_eq((int16_t)(d[5] | d[6] << 8), FULL, "torque");
    check_eq(d[7], 100, "soc scale");

    settle(0, RPM, 0.0f, 0.85f, KF_OK);
    regen_pack_debug(d);
    check_eq(d[7], 44, "partial soc scale");

    settle(0, RPM, 0.0f, 0.97f, KF_OK);
    regen_pack_debug(d);
    check_eq(d[0] & 0x08, 0, "soc scale flag clear above fade");
    check_eq(d[7], 0, "zero soc scale");
}

int main(void)
{
    test_equation();
    test_pedal();
    test_lift_into_regen();
    test_press_out_of_regen();
    test_partial_lift();
    test_boundary_dither();
    test_speed();
    test_charge_current();
    test_never_positive();
    test_peak();
    test_soc();
    test_cut();
    test_slew();
    test_pack();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
