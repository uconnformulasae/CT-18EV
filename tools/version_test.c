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

int main(void)
{
    test_pack();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
