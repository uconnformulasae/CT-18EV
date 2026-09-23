/* Host tests for adc. Includes the .c directly so the test can write the raw
 * buffers and completion flags in place of the DMA.
 *   cc -DADC_HOST -I../Core/Inc -o adc_test adc_test.c
 */

#include <stdio.h>

#include "../Core/Src/adc.c"

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

/* Stand in for the DMA completing a conversion. */
static void dma_completes(uint16_t tps1, uint16_t tps2, uint16_t bps)
{
    dual_raw = (uint32_t)tps1 | ((uint32_t)tps2 << 16);
    bps_raw = bps;
    dual_done = 1u;
    bps_done = 1u;
}

static void reset_all(void)
{
    adc_host_arm_count = 0u;
    adc_host_calibrate_count = 0u;
    adc_host_armed_before_calibration = 0u;
    adc_init();
}

static void test_init(void)
{
    printf("test_init\n");

    reset_all();
    check_eq(adc_have_samples(), 0, "no samples before the first conversion");
    check_eq(adc_tps1(), 0, "tps1 reads zero before the first conversion");
    check_eq(adc_tps2(), 0, "tps2 reads zero before the first conversion");
    check_eq(adc_bps(), 0, "bps reads zero before the first conversion");
    check_eq(adc_overruns(), 0, "no overruns yet");
    check_eq(adc_host_arm_count, 1, "init starts the first conversion");
    check_eq(adc_host_calibrate_count, 1, "init calibrates the converters");
    check_eq(adc_host_armed_before_calibration, 0,
             "calibration completes before the first conversion is armed");
}

static void test_calibration_happens_once(void)
{
    printf("test_calibration_happens_once\n");

    /* Only init calibrates; mid-run would disturb a conversion. */
    reset_all();
    for (int i = 0; i < 20; i++) {
        dma_completes(100u, 200u, 300u);
        adc_update();
    }
    check_eq(adc_host_calibrate_count, 1, "adc_update never recalibrates");
    check_eq(adc_host_arm_count, 21, "but every update re-arms");

    /* A second init calibrates again. */
    adc_init();
    check_eq(adc_host_calibrate_count, 2, "a fresh init calibrates again");
}

static void test_dual_word_is_split_correctly(void)
{
    printf("test_dual_word_is_split_correctly\n");

    /* ADC1 (TPS1) is the low half of the word, ADC2 (TPS2) the high half. */
    reset_all();
    dma_completes(1234u, 2345u, 3456u);
    adc_update();

    check_eq(adc_tps1(), 1234, "tps1 comes from the low half");
    check_eq(adc_tps2(), 2345, "tps2 comes from the high half");
    check_eq(adc_bps(), 3456, "bps comes from its own buffer");
    check_eq(adc_have_samples(), 1, "samples are available");

    /* Full scale on one channel must not bleed into the other. */
    reset_all();
    dma_completes(4095u, 0u, 0u);
    adc_update();
    check_eq(adc_tps1(), 4095, "tps1 at full scale");
    check_eq(adc_tps2(), 0, "tps2 unaffected by tps1 at full scale");

    reset_all();
    dma_completes(0u, 4095u, 0u);
    adc_update();
    check_eq(adc_tps1(), 0, "tps1 unaffected by tps2 at full scale");
    check_eq(adc_tps2(), 4095, "tps2 at full scale");
}

static void test_every_update_rearms(void)
{
    printf("test_every_update_rearms\n");

    reset_all();
    check_eq(adc_host_arm_count, 1, "armed by init");
    for (int i = 0; i < 10; i++) {
        dma_completes((uint16_t)i, (uint16_t)i, (uint16_t)i);
        adc_update();
    }
    check_eq(adc_host_arm_count, 11, "one re-arm per update");
    check_eq(adc_tps1(), 9, "latest sample latched");
}

static void test_incomplete_conversion_holds_previous(void)
{
    printf("test_incomplete_conversion_holds_previous\n");

    reset_all();
    dma_completes(1000u, 2000u, 3000u);
    adc_update();
    check_eq(adc_overruns(), 0, "no overrun on a completed conversion");

    /* Nothing completed this tick: previous readings must hold. */
    adc_update();
    check_eq(adc_tps1(), 1000, "tps1 holds its previous value");
    check_eq(adc_tps2(), 2000, "tps2 holds its previous value");
    check_eq(adc_bps(), 3000, "bps holds its previous value");
    check_eq(adc_overruns(), 1, "overrun counted");
    check_eq(adc_have_samples(), 1, "still reports having samples");

    /* Partial completion counts as an overrun too. */
    dual_done = 1u; /* only the pedal pair finished */
    adc_update();
    check_eq(adc_overruns(), 2, "a partial completion is an overrun");
    check_eq(adc_tps1(), 1000, "readings still held on partial completion");

    bps_done = 1u; /* only the brake finished */
    adc_update();
    check_eq(adc_overruns(), 3, "the other partial completion too");

    /* And it recovers. */
    dma_completes(1111u, 2222u, 3333u);
    adc_update();
    check_eq(adc_tps1(), 1111, "recovers once a conversion completes");
    check_eq(adc_overruns(), 3, "no further overruns after recovery");
}

static void test_no_overrun_before_the_first_sample(void)
{
    printf("test_no_overrun_before_the_first_sample\n");

    /* Before the first completion there is nothing to overrun. */
    reset_all();
    for (int i = 0; i < 5; i++) {
        adc_update();
    }
    check_eq(adc_overruns(), 0, "waiting for the first conversion is not an overrun");
    check_eq(adc_have_samples(), 0, "still no samples");

    dma_completes(500u, 600u, 700u);
    adc_update();
    check_eq(adc_have_samples(), 1, "first completion flips the flag");
    check_eq(adc_overruns(), 0, "and still no overruns");
}

static void test_flags_are_consumed(void)
{
    printf("test_flags_are_consumed\n");

    /* A completion is consumed once, so a dead converter stops looking alive. */
    reset_all();
    dma_completes(777u, 888u, 999u);
    adc_update();
    check_eq(adc_overruns(), 0, "first update consumes the completion");

    adc_update();
    check_eq(adc_overruns(), 1, "the same completion is not counted twice");
    check(dual_done == 0u && bps_done == 0u, "done flags cleared after each update");
}

static void test_init_clears_state(void)
{
    printf("test_init_clears_state\n");

    reset_all();
    dma_completes(1234u, 2345u, 3456u);
    adc_update();
    adc_update(); /* force an overrun */
    check(adc_overruns() > 0u, "state populated");

    reset_all();
    check_eq(adc_tps1(), 0, "init clears tps1");
    check_eq(adc_tps2(), 0, "init clears tps2");
    check_eq(adc_bps(), 0, "init clears bps");
    check_eq(adc_overruns(), 0, "init clears the overrun counter");
    check_eq(adc_errors(), 0, "init clears the error counter");
    check_eq(adc_have_samples(), 0, "init clears the samples flag");
}

int main(void)
{
    test_init();
    test_calibration_happens_once();
    test_dual_word_is_split_correctly();
    test_every_update_rearms();
    test_incomplete_conversion_holds_previous();
    test_no_overrun_before_the_first_sample();
    test_flags_are_consumed();
    test_init_clears_state();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
