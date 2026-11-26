/* Host tests for regen */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#undef REGEN_ENABLE
#define REGEN_ENABLE 1

#include "../Core/Src/regen.c"
#include "../Core/Src/bms_limits.c"

#define STEP_MS 5u
#define RPM     1500u
#define FULL    (-437)
/* The old charge clamp, 25 A per cell x 4p at 400 V, so the curve values carry over */
#define CPL_W 40000u

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

static bms_limits_t limits;

/* The BMS's limits as bms_limits_get hands them over, except that cpl_w is kept
   when not live, to show regen checks live itself */
static const bms_limits_t *lim(uint32_t cpl_w, uint8_t live)
{
    limits.dpl_w = live ? 77000u : CAR_LIMITS_LIMP_DPL_W;
    limits.cpl_w = cpl_w;
    limits.live = live;
    limits.limp = !live;
    limits.why = live ? 0u : BMS_LIMITS_WHY_STALE;
    return &limits;
}

static int32_t step(int32_t drive, uint32_t rpm, float tps)
{
    return regen_update(drive, rpm, tps, lim(CPL_W, 1u), 0u, STEP_MS);
}

static int32_t settle_l(int32_t drive, uint32_t rpm, float tps, const bms_limits_t *l)
{
    int32_t t = 0;
    for (int i = 0; i < 50; i++) {
        t = regen_update(drive, rpm, tps, l, 0u, STEP_MS);
    }
    return t;
}

static int32_t settle_c(int32_t drive, uint32_t rpm, float tps, uint32_t cpl_w, uint8_t live)
{
    return settle_l(drive, rpm, tps, lim(cpl_w, live));
}

static int32_t settle(int32_t drive, uint32_t rpm, float tps)
{
    return settle_c(drive, rpm, tps, CPL_W, 1u);
}

/* Mirrors control_torque_map in control.c */
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
            if (abs(settle(0, rpm, x) - want) > 1) {
                eq_bad++;
            }
        }

        for (uint32_t rpm = 4400; rpm <= 6000; rpm += 50) {
            const float nm = (float)CPL_W / ((float)rpm * 2.0f * 3.14159265f / 60.0f);
            const int32_t want = -(int32_t)(10.0f * p * nm);
            regen_init();
            if (abs(settle(0, rpm, x) - want) > 1) {
                clamp_bad++;
            }
        }
    }
    check_eq(eq_bad, 0, "z = -(pedal)(2y - y^2/100) below the CPL clamp");
    check_eq(clamp_bad, 0, "z = -(pedal)(CPL / w) above it");
}

static void test_pedal(void)
{
    printf("test_pedal\n");
    regen_init();
    check_eq(settle(0, RPM, 0.0f), FULL, "full regen foot off");
    check_eq(settle(0, RPM, TMAP_DEADBAND_LOW), FULL, "held below low deadband");

    const float mid = (TMAP_DEADBAND_LOW + TMAP_REGEN_END) / 2.0f;
    const int32_t half = settle(0, RPM, mid);
    check(half >= -219 && half <= -217, "half regen mid pedal");

    check_eq(settle(0, RPM, TMAP_REGEN_END), 0, "zero at regen end");
    check_eq(settle(0, RPM, 0.37f), 0, "zero in coast band");
    check_eq(settle(0, RPM, 1.1f), 0, "zero at full pedal with no drive");
    check_eq(settle(0, RPM, -0.1f), FULL, "negative tps clamps to full regen");
}

static void test_lift_into_regen(void)
{
    printf("test_lift_into_regen\n");
    regen_init();
    int32_t last = settle(drive_for(0.6f), RPM, 0.6f);
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
    int32_t last = settle(0, RPM, 0.0f);

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
    settle(drive_for(0.6f), RPM, 0.6f);
    check_eq(settle(0, RPM, 0.15f), -324, "partial regen at 15% pedal");
    check_eq(settle(0, RPM, 0.37f), 0, "coast at 37% pedal");
    check_eq(settle(drive_for(0.6f), RPM, 0.6f), drive_for(0.6f), "back to drive");
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
    check_eq(settle(0, 0, 0.0f), 0, "none at standstill");
    check_eq(settle(0, REGEN_CUTOFF_RPM, 0.0f), 0, "none at cutoff");

    const int32_t a = settle(0, 520, 0.0f);
    const int32_t b = settle(0, 600, 0.0f);
    const int32_t c = settle(0, REGEN_RAMP_RPM, 0.0f);
    check(a < 0 && a > b && b > c, "ramps in above cutoff");

    check_eq(settle(0, 1000, 0.0f), -305, "equation at 1000 rpm");
    check_eq(settle(0, 2000, 0.0f), -555, "equation at 2000 rpm");
    check_eq(settle(0, 3000, 0.0f), -750, "equation at 3000 rpm");
    check_eq(settle(0, 6000, 0.0f), -636, "CPL clamp at max rpm");
    check_eq(settle(0, 12000, 0.0f), -318, "CPL clamp past max rpm");
}

static void test_charge_power(void)
{
    printf("test_charge_power\n");
    const uint32_t cpls[] = {10000u, 20000u, 35000u, 40000u, 60000u};
    int over = 0, unused = 0;
    for (size_t k = 0; k < sizeof(cpls) / sizeof(cpls[0]); k++) {
        float worst = 0.0f;
        for (uint32_t rpm = REGEN_RAMP_RPM; rpm <= 9000; rpm += 50) {
            regen_init();
            const int32_t t = settle_c(0, rpm, 0.0f, cpls[k], 1u);
            const float ratio = (float)-t / 10.0f * (float)rpm * RPM_TO_RAD_S / (float)cpls[k];
            if (ratio > worst) {
                worst = ratio;
            }
        }
        over += worst > 1.0f;
        unused += worst <= 0.99f;
    }
    check_eq(over, 0, "shaft power within CPL (efficiency 1)");
    check_eq(unused, 0, "and uses all of it");

    regen_init();
    check_eq(settle_c(0, 6000, 0.0f, 35000u, 1u), -557, "clamp follows CPL");
}

static void test_never_positive(void)
{
    printf("test_never_positive\n");
    int bad = 0;
    for (int i = -10; i <= 110; i++) {
        for (uint32_t rpm = 0; rpm <= 15000; rpm += 125) {
            regen_init();
            const int32_t t = settle(0, rpm, (float)i / 100.0f);
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
    check_eq(settle_c(0, (uint32_t)REGEN_RPM_MAX, 0.0f, 200000u, 1u),
             -(int32_t)(10.0f * REGEN_PEAK_NM), "peak at max rpm without the CPL clamp");
}

static void test_limits(void)
{
    printf("test_limits\n");
    regen_init();
    check_eq(settle_c(0, RPM, 0.0f, 0u, 1u), 0, "no regen at CPL 0");
    check_eq(settle_c(0, 3000, 0.0f, 0u, 1u), 0, "nor at speed");

    /* 20 kW / (3000 rpm * 2 pi / 60) = 63.66 Nm, under the curve's 75 Nm */
    regen_init();
    check_eq(settle_c(0, 3000, 0.0f, 20000u, 1u), -636, "20 kW at 3000 rpm");
    check(fabsf(regen_get_debug()->max_nm - 63.66f) < 0.01f, "max regen 63.66 Nm");
    check(regen_get_debug()->max_nm < 75.0f, "charge-limited, below the peak curve");
    check_eq(regen_get_debug()->limits_live, 1, "limits live");

    check_eq(settle_c(0, 3000, 0.0f, 20000u, 0u), 0, "off when the limits are not live");
    check_eq(regen_get_debug()->limits_live, 0, "limits not live");
    check_eq(settle_c(0, 3000, 0.0f, 20000u, 1u), -636, "back when live");

    /* Through bms_limits, as main.c feeds it */
    bms_limits_t l;
    bms_limits_init();
    bms_limits_get(0u, &l);
    check_eq(l.cpl_w, 0, "nothing received: limp, CPL 0");
    regen_init();
    check_eq(settle_l(0, 3000, 0.0f, &l), 0, "no regen in limp");

    uint8_t d[8];
    const car_limits_t frame = {.dpl_w = 77000u, .cpl_w = 20000u, .flags = 0u, .counter = 1u};
    car_limits_pack(&frame, d);
    bms_limits_feed(d, CAR_LIMITS_DLC, 1000u);
    bms_limits_get(1000u, &l);
    check_eq(l.live, 1, "fresh frame");
    check_eq(settle_l(0, 3000, 0.0f, &l), -636, "regen from the BMS's CPL");

    bms_limits_get(1000u + BMS_LIMITS_STALE_MS + 1u, &l);
    check_eq(l.live, 0, "frame gone stale");
    check_eq(settle_l(0, 3000, 0.0f, &l), 0, "regen stops");
}

static void test_cut(void)
{
    printf("test_cut\n");
    regen_init();
    settle(0, RPM, 0.0f);
    const bms_limits_t *ok = lim(CPL_W, 1u);
    check_eq(regen_update(0, RPM, 0.0f, ok, 1u, STEP_MS), 0, "cut drops regen at once");
    check_eq(regen_update(500, RPM, 0.9f, ok, 1u, STEP_MS), 500, "cut passes drive");
    check_eq(regen_update(0, RPM, 0.0f, ok, 1u, STEP_MS), 0, "cut from drive");
    check_eq(step(0, RPM, 0.0f), -100, "ramps back in");
    check_eq(settle(0, RPM, 0.0f), FULL, "settles");
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
    settle(0, RPM, 0.0f);
    check_eq(regen_update(1000, RPM, 0.9f, lim(CPL_W, 1u), 0u, 1000u), FULL + 150,
             "long dt is clamped");
}

static void test_pack(void)
{
    printf("test_pack\n");
    regen_init();
    settle(0, RPM, 0.0f);

    uint8_t d[8];
    memset(d, 0xAA, sizeof(d));
    regen_pack_debug(d);
    check_eq(d[0], 0, "not cut");
    check_eq(d[1], 40, "CPL (kW)");
    check_eq(d[2], 43, "max regen (Nm)");
    check(regen_get_debug() == &dbg, "debug accessor");
    check_eq(regen_get_debug()->cut, d[0], "accessor matches the frame");
    for (int i = REGEN_DEBUG_DLC; i < 8; i++) {
        check_eq(d[i], 0xAA, "nothing written past dlc");
    }

    settle_c(0, RPM, 0.0f, 23999u, 1u);
    regen_pack_debug(d);
    check_eq(d[1], 23, "CPL rounds down");

    settle_c(0, RPM, 0.0f, 300000u, 1u);
    regen_pack_debug(d);
    check_eq(d[1], 255, "CPL saturates");

    settle_c(0, RPM, 0.0f, 20000u, 0u);
    regen_pack_debug(d);
    check_eq(d[1], 0, "no CPL when not live");

    regen_update(1000, RPM, 0.9f, lim(CPL_W, 1u), 1u, STEP_MS);
    regen_pack_debug(d);
    check_eq(d[0], 1, "cut");
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
    test_charge_power();
    test_never_positive();
    test_peak();
    test_limits();
    test_cut();
    test_slew();
    test_pack();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
