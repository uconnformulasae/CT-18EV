/* Host tests for bms_limits */

#include <stdio.h>
#include <string.h>

#include "../Core/Src/bms_limits.c"

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

/* The shared vector: DPL 77000, CPL 23400, no flags, counter 7, version 1 */
static const uint8_t vector[8] = {0x02, 0x03, 0xEA, 0x00, 0x00, 0x00, 0x07, 0x01};

static void frame(uint8_t counter, uint8_t flags, uint8_t version, uint32_t tick)
{
    uint8_t d[8];
    memcpy(d, vector, sizeof d);
    d[4] = flags;
    d[6] = counter;
    d[7] = version;
    bms_limits_feed(d, 8u, tick);
}

static bms_limits_t get(uint32_t tick)
{
    bms_limits_t o;
    bms_limits_get(tick, &o);
    return o;
}

static void test_shared_vector(void)
{
    printf("test_shared_vector\n");
    car_limits_t l = {.dpl_w = 77000u, .cpl_w = 23400u, .flags = 0u, .counter = 7u};
    uint8_t d[8];
    car_limits_pack(&l, d);
    check(memcmp(d, vector, 8) == 0, "packs to the shared bytes");
    car_limits_t u = {0};
    check(car_limits_unpack(vector, &u), "unpacks");
    check_eq((long)u.dpl_w, 77000, "dpl");
    check_eq((long)u.cpl_w, 23400, "cpl");
    check_eq(u.counter, 7, "counter");
    uint8_t v2[8];
    memcpy(v2, vector, 8);
    v2[7] = 2u;
    check(!car_limits_unpack(v2, &u), "another version is refused");
    check_eq(CAR_LIMITS_ID, 0x203, "id");
    check_eq(CAR_LIMITS_LIMP_DPL_W, 52500, "limp dpl");
}

static void test_no_frame_is_limp(void)
{
    printf("test_no_frame_is_limp\n");
    bms_limits_init();
    bms_limits_t o = get(5000u);
    check_eq(o.live, 0, "not live");
    check_eq(o.limp, 1, "limp");
    check_eq(o.why, BMS_LIMITS_WHY_STALE, "why stale");
    check_eq((long)o.dpl_w, 52500, "limp dpl");
    check_eq((long)o.cpl_w, 0, "limp cpl");
}

static void test_good_frame_is_live_for_100ms(void)
{
    printf("test_good_frame_is_live_for_100ms\n");
    bms_limits_init();
    frame(7u, 0u, 1u, 1000u);
    bms_limits_t o = get(1000u);
    check_eq(o.live, 1, "live at 1000");
    check_eq(o.limp, 0, "not limp");
    check_eq(o.why, 0, "no why");
    check_eq((long)o.dpl_w, 77000, "dpl");
    check_eq((long)o.cpl_w, 23400, "cpl");
    o = get(1100u);
    check_eq(o.live, 1, "live at 1100");
    check_eq((long)o.dpl_w, 77000, "dpl at 1100");
    o = get(1101u);
    check_eq(o.live, 0, "stale at 1101");
    check_eq(o.limp, 1, "limp at 1101");
    check_eq(o.why, BMS_LIMITS_WHY_STALE, "why stale");
    check_eq((long)o.dpl_w, 52500, "limp dpl");
    check_eq((long)o.cpl_w, 0, "limp cpl");
}

static void test_short_frame_is_ignored(void)
{
    printf("test_short_frame_is_ignored\n");
    bms_limits_init();
    bms_limits_feed(vector, 7u, 1000u);
    bms_limits_t o = get(1000u);
    check_eq(o.live, 0, "DLC 7 ignored");
    check_eq(o.why, BMS_LIMITS_WHY_STALE, "no extra why");
}

static void test_other_version_is_ignored(void)
{
    printf("test_other_version_is_ignored\n");
    bms_limits_init();
    frame(7u, 0u, 2u, 1000u);
    bms_limits_t o = get(1000u);
    check_eq(o.live, 0, "version 2 ignored");
    check(o.why & BMS_LIMITS_WHY_VERSION, "why version");
    frame(8u, 0u, 1u, 1010u);
    o = get(1010u);
    check_eq(o.live, 1, "a good frame recovers");
    check_eq(o.why, 0, "why cleared");
}

static void test_repeated_counter_does_not_refresh(void)
{
    printf("test_repeated_counter_does_not_refresh\n");
    bms_limits_init();
    frame(7u, 0u, 1u, 1000u);
    frame(8u, 0u, 1u, 1020u);
    for (uint32_t t = 1040u; t <= 1200u; t += 20u) {
        frame(8u, 0u, 1u, t);
    }
    bms_limits_t o = get(1120u);
    check_eq(o.live, 1, "live 100 ms after the last changed counter");
    o = get(1121u);
    check_eq(o.live, 0, "limp past 100 ms");
    check_eq(o.limp, 1, "limp");
    check(o.why & BMS_LIMITS_WHY_COUNTER, "why counter");
    check(o.why & BMS_LIMITS_WHY_STALE, "and stale");
    frame(9u, 0u, 1u, 1220u);
    o = get(1220u);
    check_eq(o.live, 1, "a changed counter recovers");
}

static void test_counter_wraps(void)
{
    printf("test_counter_wraps\n");
    bms_limits_init();
    frame(255u, 0u, 1u, 1000u);
    frame(0u, 0u, 1u, 1020u);
    bms_limits_t o = get(1120u);
    check_eq(o.live, 1, "255 to 0 accepted: live 100 ms after the wrap frame");
    o = get(1121u);
    check_eq(o.live, 0, "and stale after");
}

static void test_tick_wraps(void)
{
    printf("test_tick_wraps\n");
    bms_limits_init();
    frame(7u, 0u, 1u, 0xFFFFFFC0u);
    bms_limits_t o = get(0x00000010u);
    check_eq(o.live, 1, "live across the tick wrap");
    check_eq((long)o.dpl_w, 77000, "dpl");
}

static void test_frame_stamped_after_the_callers_tick_is_live(void)
{
    printf("test_frame_stamped_after_the_callers_tick_is_live\n");
    bms_limits_init();
    frame(7u, 0u, 1u, 1001u);
    bms_limits_t o = get(1000u);
    check_eq(o.live, 1, "live");
    check_eq((long)o.dpl_w, 77000, "dpl as fed");
    check_eq(o.why, 0, "no why");
}

static void test_limp_flag_in_a_fresh_frame(void)
{
    printf("test_limp_flag_in_a_fresh_frame\n");
    bms_limits_init();
    frame(7u, CAR_LIMITS_FLAG_LIMP, 1u, 1000u);
    bms_limits_t o = get(1000u);
    check_eq(o.live, 1, "live");
    check_eq(o.limp, 1, "limp");
    check_eq(o.why, 0, "no why: the BMS chose it");
}

int main(void)
{
    test_shared_vector();
    test_no_frame_is_limp();
    test_good_frame_is_live_for_100ms();
    test_short_frame_is_ignored();
    test_other_version_is_ignored();
    test_repeated_counter_does_not_refresh();
    test_counter_wraps();
    test_tick_wraps();
    test_frame_stamped_after_the_callers_tick_is_live();
    test_limp_flag_in_a_fresh_frame();

    printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
