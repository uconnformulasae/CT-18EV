/* Host tests for regen built with REGEN_ENABLE 0 */

#include <stdio.h>
#include <string.h>

#undef REGEN_ENABLE
#define REGEN_ENABLE 0

#include "../Core/Src/regen.c"
#include "../Core/Src/soc_kf.c"

#define KF_OK   (SOC_KF_FLAG_INIT | SOC_KF_FLAG_BMS_LIVE)
#define STEP_MS 5u

static int failures = 0;
static int checks = 0;

static void check_eq(long got, long want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL: %s (got %ld want %ld)\n", what, got, want);
    }
}

static void test_passthrough(void)
{
    printf("test_passthrough\n");
    soc_kf_debug_t kf = {.soc = 0.5f, .v_pack = 400.0f, .flags = KF_OK};

    regen_init();
    check_eq(regen_update(1500, 3000u, 0.8f, &kf, 0u, STEP_MS), 1500, "drive torque untouched");
    check_eq(regen_update(0, 3000u, 0.0f, &kf, 0u, STEP_MS), 0, "no regen on foot off");
    check_eq(regen_get_debug()->target, 0, "target is the drive torque");
    check_eq(regen_get_debug()->torque, 0, "torque is the drive torque");
    check_eq(regen_get_debug()->cut, 1, "reports cut");
}

static void test_pack(void)
{
    printf("test_pack\n");
    uint8_t d[8];
    memset(d, 0xAA, sizeof(d));
    regen_pack_debug(d);
    for (int i = 0; i < REGEN_DEBUG_DLC; i++) {
        check_eq(d[i], 0, "frame zeroed");
    }
    for (int i = REGEN_DEBUG_DLC; i < 8; i++) {
        check_eq(d[i], 0xAA, "nothing written past dlc");
    }
}

int main(void)
{
    test_passthrough();
    test_pack();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
