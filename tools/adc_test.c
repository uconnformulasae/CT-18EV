/* Host tests for adc */

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
    check_eq(adc_have_samples(), 0, "no samples before first conversion");
    check_eq(adc_tps1(), 0, "tps1 zero");
    check_eq(adc_tps2(), 0, "tps2 zero");
    check_eq(adc_bps(), 0, "bps zero");
    check_eq(adc_overruns(), 0, "no overruns");
    check_eq(adc_host_arm_count, 1, "arm count 1");
    check_eq(adc_host_calibrate_count, 1, "calibrate count 1");
    check_eq(adc_host_armed_before_calibration, 0, "calibrated before arm");
}

static void test_calibration_happens_once(void)
{
    printf("test_calibration_happens_once\n");

    reset_all();
    for (int i = 0; i < 20; i++) {
        dma_completes(100u, 200u, 300u);
        adc_update();
    }
    check_eq(adc_host_calibrate_count, 1, "no recalibration during update");
    check_eq(adc_host_arm_count, 21, "re-arms every update");

    adc_init();
    check_eq(adc_host_calibrate_count, 2, "re-calibrates on init");
}

static void test_dual_word_is_split_correctly(void)
{
    printf("test_dual_word_is_split_correctly\n");

    reset_all();
    dma_completes(1234u, 2345u, 3456u);
    adc_update();

    check_eq(adc_tps1(), 1234, "tps1 lower half");
    check_eq(adc_tps2(), 2345, "tps2 upper half");
    check_eq(adc_bps(), 3456, "bps buffer");
    check_eq(adc_have_samples(), 1, "have samples");

    /* Channel isolation at full scale */
    reset_all();
    dma_completes(4095u, 0u, 0u);
    adc_update();
    check_eq(adc_tps1(), 4095, "tps1 full scale");
    check_eq(adc_tps2(), 0, "tps2 zero");

    reset_all();
    dma_completes(0u, 4095u, 0u);
    adc_update();
    check_eq(adc_tps1(), 0, "tps1 zero");
    check_eq(adc_tps2(), 4095, "tps2 full scale");
}

static void test_incomplete_conversion_holds_previous(void)
{
    printf("test_incomplete_conversion_holds_previous\n");

    reset_all();
    dma_completes(1000u, 2000u, 3000u);
    adc_update();
    check_eq(adc_overruns(), 0, "no overrun on completed conversion");

    /* Missing conversion holds previous values */
    adc_update();
    check_eq(adc_tps1(), 1000, "tps1 held");
    check_eq(adc_tps2(), 2000, "tps2 held");
    check_eq(adc_bps(), 3000, "bps held");
    check_eq(adc_overruns(), 1, "overrun incremented");
    check_eq(adc_have_samples(), 1, "still has samples");

    /* Partial completion */
    dual_done = 1u;
    adc_update();
    check_eq(adc_overruns(), 2, "partial completion overrun");
    check_eq(adc_tps1(), 1000, "readings held on partial completion");

    bps_done = 1u;
    adc_update();
    check_eq(adc_overruns(), 3, "other partial completion overrun");

    /* Recovery */
    dma_completes(1111u, 2222u, 3333u);
    adc_update();
    check_eq(adc_tps1(), 1111, "recovers on full completion");
    check_eq(adc_overruns(), 3, "no further overruns");
}

static void test_no_overrun_before_the_first_sample(void)
{
    printf("test_no_overrun_before_the_first_sample\n");

    reset_all();
    for (int i = 0; i < 5; i++) {
        adc_update();
    }
    check_eq(adc_overruns(), 0, "no overrun before first completion");
    check_eq(adc_have_samples(), 0, "no samples yet");

    dma_completes(500u, 600u, 700u);
    adc_update();
    check_eq(adc_have_samples(), 1, "first completion sets sample flag");
    check_eq(adc_overruns(), 0, "still no overruns");
}

static void test_init_clears_state(void)
{
    printf("test_init_clears_state\n");

    reset_all();
    dma_completes(1234u, 2345u, 3456u);
    adc_update();
    adc_update();
    check(adc_overruns() > 0u, "state populated");

    reset_all();
    check_eq(adc_tps1(), 0, "clears tps1");
    check_eq(adc_tps2(), 0, "clears tps2");
    check_eq(adc_bps(), 0, "clears bps");
    check_eq(adc_overruns(), 0, "clears overruns");
    check_eq(adc_errors(), 0, "clears errors");
    check_eq(adc_have_samples(), 0, "clears samples flag");
}

int main(void)
{
    test_init();
    test_calibration_happens_once();
    test_dual_word_is_split_correctly();
    test_incomplete_conversion_holds_previous();
    test_no_overrun_before_the_first_sample();
    test_init_clears_state();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
