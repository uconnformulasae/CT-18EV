/* Host tests for version */

#include <stdio.h>

#include "../Core/Src/version.c"

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

static void test_pack(void)
{
    printf("test_pack\n");
    const uint32_t hash = (uint32_t)BUILD_HASH;
    uint8_t d[8] = {0};

    version_pack(d);
    check_eq(d[0] | d[1] << 8, (long)(hash >> 16), "hash high half, little endian");
    check_eq(d[2] | d[3] << 8, (long)(hash & 0xFFFF), "hash low half, little endian");
    check_eq(d[4], BUILD_FLAGS, "flags");
    check_eq(VERSION_DLC, 5, "dlc covers what is packed");
}

/* Steps 5 ms loops from start for ms, returns frames sent */
static int run(version_tx_t *v, uint32_t start, uint32_t ms, uint32_t *first, uint32_t *last)
{
    int sent = 0;
    for (uint32_t t = 0; t <= ms; t += 5u) {
        if (version_tx_due(v, start + t)) {
            if (sent == 0) {
                *first = t;
            }
            *last = t;
            sent++;
        }
    }
    return sent;
}

static void test_boot_burst(void)
{
    printf("test_boot_burst\n");
    version_tx_t v;
    uint32_t first = 0, last = 0;
    const uint32_t burst_end = (VERSION_TX_COUNT - 1) * VERSION_TX_MS;

    version_tx_init(&v, 1234u);
    check_eq(run(&v, 1234u, burst_end + VERSION_REPEAT_MS - 5u, &first, &last), VERSION_TX_COUNT,
             "burst, then quiet until the first repeat");
    check_eq(first, 0, "first frame right away");
    check_eq(last, burst_end, "burst one per period");

    version_tx_init(&v, 0u);
    check_eq(version_tx_due(&v, 0u), 1, "due at boot");
    check_eq(version_tx_due(&v, VERSION_TX_MS - 1u), 0, "not due inside a period");
    check_eq(version_tx_due(&v, VERSION_TX_MS), 1, "due after a period");
}

static void test_repeat(void)
{
    printf("test_repeat\n");
    version_tx_t v;
    uint32_t first = 0, last = 0;
    const uint32_t burst_end = (VERSION_TX_COUNT - 1) * VERSION_TX_MS;

    version_tx_init(&v, 0u);
    check_eq(run(&v, 0u, burst_end + 3u * VERSION_REPEAT_MS, &first, &last), VERSION_TX_COUNT + 3,
             "keeps repeating after the burst");
    check_eq(last, burst_end + 3u * VERSION_REPEAT_MS, "repeat spacing");

    /* Repeats never stop, even past the uint8_t counter range */
    version_tx_init(&v, 0u);
    check_eq(run(&v, 0u, burst_end + 300u * VERSION_REPEAT_MS, &first, &last),
             VERSION_TX_COUNT + 300, "repeats forever");
}

static void test_boot_burst_wrap(void)
{
    printf("test_boot_burst_wrap\n");
    version_tx_t v;
    uint32_t first = 0, last = 0;
    const uint32_t start = UINT32_MAX - 2500u;
    const uint32_t burst_end = (VERSION_TX_COUNT - 1) * VERSION_TX_MS;

    version_tx_init(&v, start);
    check_eq(run(&v, start, burst_end + VERSION_REPEAT_MS, &first, &last), VERSION_TX_COUNT + 1,
             "tick wrap mid burst");
    check_eq(last, burst_end + VERSION_REPEAT_MS, "spacing survives the wrap");
}

int main(void)
{
    test_pack();
    test_boot_burst();
    test_repeat();
    test_boot_burst_wrap();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
