/* Host tests for rtd */

#include <stdio.h>

#include "../Core/Src/rtd.c"

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

/* Return elapsed ms when debounce latches, or 0 if timed out */
static uint32_t latch_time(uint32_t dt_ms, uint32_t budget_ms)
{
    rtd_init();
    for (uint32_t t = 0; t < budget_ms; t += dt_ms) {
        if (rtd_debounce_update(1u, dt_ms)) {
            return t + dt_ms;
        }
    }
    return 0u;
}

static void test_latches_after_the_hold_time(void)
{
    printf("test_latches_after_the_hold_time\n");

    rtd_init();
    check_eq(rtd_hold_ms(), 0, "starts empty");
    check_eq(rtd_debounce_update(1u, 10u), 0, "not latched immediately");
    check_eq(rtd_hold_ms(), 10, "hold time accumulates in ms");

    rtd_init();
    for (uint32_t t = 0; t < RTD_DEBOUNCE_TRIP_MS; t += 10u) {
        check_eq(rtd_debounce_update(1u, 10u), 0, "not latched before the trip");
    }

    check_eq(rtd_debounce_update(1u, 10u), 1, "latches just past the trip");
}

static void test_hold_time_is_step_size_invariant(void)
{
    printf("test_hold_time_is_step_size_invariant\n");

    const uint32_t steps[] = {1u, 2u, 5u, 10u, 11u, 25u};

    for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        const uint32_t dt = steps[i];
        const uint32_t t = latch_time(dt, 2000u);
        printf("    %2u ms steps -> latched at %3u ms\n", dt, t);
        check(t > RTD_DEBOUNCE_TRIP_MS, "latched no earlier than the trip time");
        check(t <= RTD_DEBOUNCE_TRIP_MS + dt, "latched within one step of the trip time");
    }
}

static void test_release_drains_faster_than_hold_fills(void)
{
    printf("test_release_drains_faster_than_hold_fills\n");

    /* Release drains faster than hold fills */
    rtd_init();
    for (int i = 0; i < 10; i++) {
        rtd_debounce_update(1u, 10u);
    }
    const uint32_t filled = rtd_hold_ms();
    check_eq(filled, 100, "100 ms of hold accumulated");

    for (int i = 0; i < 5; i++) {
        rtd_debounce_update(0u, 10u);
    }
    check(rtd_hold_ms() < filled - 50u, "release drains faster than 1:1");
    check_eq(rtd_hold_ms(),
             (100 * RTD_DEBOUNCE_FALL_DEN - 50 * RTD_DEBOUNCE_FALL_NUM) / RTD_DEBOUNCE_FALL_DEN,
             "drains at exactly NUM/DEN");

    /* Floor at zero */
    for (int i = 0; i < 100; i++) {
        rtd_debounce_update(0u, 10u);
    }
    check_eq(rtd_hold_ms(), 0, "accumulator floors at zero");
}

static void test_chatter_cannot_latch(void)
{
    printf("test_chatter_cannot_latch\n");

    /* 50% duty cycle never latches */
    const uint32_t steps[] = {1u, 5u, 10u};
    for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        rtd_init();
        int latched = 0;
        for (int n = 0; n < 4000; n++) {
            if (rtd_debounce_update((uint8_t)(n & 1), steps[i])) {
                latched = 1;
            }
        }
        check(!latched, "50% duty chatter never latches");
        check(rtd_hold_ms() <= steps[i], "50% duty chatter banks at most one step");
    }

    /* High duty cycle latches */
    rtd_init();
    int latched = 0;
    for (int n = 0; n < 4000 && !latched; n++) {
        latched = rtd_debounce_update((uint8_t)((n % 10) != 0), 1u); /* 90% duty */
    }
    check(latched, "90% duty does latch");
}

static void test_accumulator_is_capped(void)
{
    printf("test_accumulator_is_capped\n");

    rtd_init();
    for (int i = 0; i < 500; i++) {
        rtd_debounce_update(1u, 10u);
    }
    check_eq(rtd_hold_ms(), RTD_DEBOUNCE_MAX_MS, "hold caps at the ceiling");

    /* Drains in bounded time */
    int steps_to_zero = 0;
    while (rtd_hold_ms() > 0u && steps_to_zero < 10000) {
        rtd_debounce_update(0u, 10u);
        steps_to_zero++;
    }
    check(steps_to_zero < 40, "a full accumulator drains in well under half a second");
}

static void test_stall_does_not_jump_the_accumulator(void)
{
    printf("test_stall_does_not_jump_the_accumulator\n");

    /* Stall clamped to max dt */
    rtd_init();
    check_eq(rtd_debounce_update(1u, 5000u), 0, "a 5 s step does not latch on its own");
    check_eq(rtd_hold_ms(), RTD_DEBOUNCE_DT_MAX_MS, "the step is clamped to the dt ceiling");
}

static void test_init_clears_state(void)
{
    printf("test_init_clears_state\n");

    rtd_init();
    for (int i = 0; i < 50; i++) {
        rtd_debounce_update(1u, 10u);
    }
    check(rtd_hold_ms() > 0u, "accumulator populated");
    rtd_init();
    check_eq(rtd_hold_ms(), 0, "init clears the accumulator");
    check_eq(rtd_debounce_update(1u, 10u), 0, "and the latch starts over");
}

int main(void)
{
    test_latches_after_the_hold_time();
    test_hold_time_is_step_size_invariant();
    test_release_drains_faster_than_hold_fills();
    test_chatter_cannot_latch();
    test_accumulator_is_capped();
    test_stall_does_not_jump_the_accumulator();
    test_init_clears_state();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
