/* Host tests for inv_tune */

#include <stdio.h>
#include <string.h>

#include "../Core/Src/inv_tune.c"

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

#define LOOP_MS 5u

/* Fake inverter */
static uint32_t now;
static uint8_t enabled;
static uint8_t answering;
static uint8_t heartbeat;
static uint16_t value_offset;
static uint16_t skip_addr; /* Never answered */
static uint32_t requests;
static uint8_t wrote; /* Any request that was not a plain read */

static uint16_t fake_value(uint16_t addr)
{
    return (uint16_t)(addr * 3u + value_offset);
}

static void reply(uint16_t addr, uint16_t value)
{
    const uint8_t d[8] = {(uint8_t)(addr & 0xFF),  (uint8_t)(addr >> 8),  0, 0,
                          (uint8_t)(value & 0xFF), (uint8_t)(value >> 8), 0, 0};
    inv_tune_feed_response(d);
}

static void reset(void)
{
    inv_tune_init();
    now = 1000u;
    enabled = 0u;
    answering = 1u;
    heartbeat = 1u;
    value_offset = 0u;
    skip_addr = 0u;
    requests = 0u;
    wrote = 0u;
}

/* One 5 ms control loop. Returns the address requested, or 0. */
static uint16_t step(void)
{
    uint8_t req[8];

    now += LOOP_MS;
    if (heartbeat) {
        inv_tune_feed_state(now);
    }
    memset(req, 0xEE, sizeof(req));
    if (!inv_tune_update(now, enabled, req)) {
        return 0u;
    }

    requests++;
    const uint16_t addr = (uint16_t)(req[0] | req[1] << 8);
    if (req[2] != 0u || req[3] || req[4] || req[5] || req[6] || req[7]) {
        wrote = 1u;
    }
    if (answering && addr != skip_addr) {
        reply(addr, fake_value(addr));
    }
    return addr;
}

static void run_ms(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += LOOP_MS) {
        step();
    }
}

/* Run until the sweep finishes, returns elapsed ms or 0 on timeout */
static uint32_t run_until_done(uint32_t budget_ms)
{
    for (uint32_t t = 0; t < budget_ms; t += LOOP_MS) {
        step();
        if (inv_tune_state() == INV_TUNE_DONE) {
            return t + LOOP_MS;
        }
    }
    return 0u;
}

static void test_table(void)
{
    printf("test_table\n");

    check_eq(inv_tune_count(), 92, "every documented export parameter");
    check_eq(inv_tune_addrs[84], 251, "original 85 keep their slots");

    uint8_t unique = 1u;
    uint8_t in_range = 1u;
    for (uint8_t i = 0; i < inv_tune_count(); i++) {
        /* 22 is the AIN pull-up, a general parameter */
        if ((inv_tune_addrs[i] < 100u || inv_tune_addrs[i] > 499u) && inv_tune_addrs[i] != 22u) {
            in_range = 0u;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (inv_tune_addrs[i] == inv_tune_addrs[j]) {
                unique = 0u;
            }
        }
    }
    check(unique, "no address read twice");
    check(in_range, "all in the user EEPROM range");

    check(2u * INV_TUNE_TX_MS < 500u, "both frames repeat faster than the AiM timeout");
}

static void test_waits_for_inverter(void)
{
    printf("test_waits_for_inverter\n");

    reset();
    heartbeat = 0u;
    run_ms(5000u);
    check_eq(requests, 0, "nothing sent with no inverter");
    check_eq(inv_tune_state(), INV_TUNE_WAITING, "waiting");

    heartbeat = 1u;
    step();
    check_eq(inv_tune_state(), INV_TUNE_SETTLING, "settling once 0x0AA arrives");
    run_ms(INV_TUNE_SETTLE_MS - 2u * LOOP_MS);
    check_eq(requests, 0, "nothing sent while the inverter boots");

    uint16_t first = 0u;
    for (int i = 0; i < 3 && !first; i++) {
        first = step();
    }
    check_eq(first, 174, "first read is the serial number");
    check_eq(inv_tune_state(), INV_TUNE_READING, "reading");
}

static void test_full_sweep(void)
{
    printf("test_full_sweep\n");

    reset();
    const uint32_t ms = run_until_done(10000u);
    check(ms > 0u, "sweep finishes");
    check(ms < INV_TUNE_SETTLE_MS + 500u, "one parameter per loop, about half a second");
    check_eq(requests, inv_tune_count(), "one request per parameter when all answer");
    check(!wrote, "only ever reads");

    uint8_t all = 1u;
    for (uint8_t i = 0; i < inv_tune_count(); i++) {
        if (inv_tune_value(i) != fake_value(inv_tune_addrs[i])) {
            all = 0u;
        }
    }
    check(all, "every value stored in table order");

    run_ms(5000u);
    check_eq(requests, inv_tune_count(), "no more reads once done");
}

static void test_ignores_other_addresses(void)
{
    printf("test_ignores_other_addresses\n");

    reset();
    answering = 0u;
    run_ms(INV_TUNE_SETTLE_MS + 2u * LOOP_MS);
    check_eq(requests, 1, "first request out");

    reply(0u, 1234u); /* Unrecognized address */
    step();
    check_eq(requests, 1, "0,0 reply is not an answer");
    reply(175u, 1234u);
    step();
    check_eq(requests, 1, "wrong address is not an answer");
    reply(174u, 2027u);
    step();
    check_eq(requests, 2, "matching reply moves on");
    check_eq(inv_tune_value(0), 2027, "serial stored");
}

static void test_retries_then_missing(void)
{
    printf("test_retries_then_missing\n");

    reset();
    skip_addr = 241u; /* Second parameter */
    const uint32_t ms = run_until_done(20000u);
    check(ms > 0u, "sweep still finishes");
    check_eq(requests, inv_tune_count() + INV_TUNE_RETRIES, "retried before giving up");
    check_eq(inv_tune_value(1), INV_TUNE_MISSING, "silent parameter marked missing");
    check_eq(inv_tune_value(2), fake_value(246u), "next parameter still read");

    uint16_t id;
    uint8_t d[8];
    inv_tune_pack_next(&id, d);
    check_eq(id, INV_TUNE_CAN_ID_STATUS, "status first");
    check_eq(d[1], inv_tune_count() - 1, "read count");
    check_eq(d[2], 1, "missing count");

    inv_tune_pack_next(&id, d);
    inv_tune_pack_next(&id, d);
    inv_tune_pack_next(&id, d);
    check_eq(id, INV_TUNE_CAN_ID_VALUE, "second value frame");
    check_eq(d[0] | d[1] << 8, 241, "address of the silent parameter");
    check_eq(d[2] | d[3] << 8, INV_TUNE_MISSING, "sent as 0x8000");
}

static void test_pauses_while_enabled(void)
{
    printf("test_pauses_while_enabled\n");

    reset();
    run_ms(INV_TUNE_SETTLE_MS + 100u);
    const uint32_t before = requests;
    check(before > 0u, "reading");

    enabled = 1u;
    run_ms(2000u);
    check(requests <= before + 1u, "at most the in-flight read finishes");
    check_eq(inv_tune_state(), INV_TUNE_PAUSED, "paused");

    enabled = 0u;
    check(run_until_done(2000u) > 0u, "resumes and finishes");
    check_eq(requests, inv_tune_count(), "no parameter read twice across the pause");
}

static void test_rereads_after_inverter_restart(void)
{
    printf("test_rereads_after_inverter_restart\n");

    reset();
    run_until_done(10000u);
    const uint16_t crc_before = inv_tune_crc();

    heartbeat = 0u;
    run_ms(INV_TUNE_LOST_MS + 50u);
    check_eq(inv_tune_state(), INV_TUNE_WAITING, "inverter gone");

    uint16_t id;
    uint8_t d[8];
    uint8_t frames = 0u;
    for (uint16_t i = 0; i < 2u * inv_tune_count(); i++) {
        if (inv_tune_pack_next(&id, d) && id == INV_TUNE_CAN_ID_VALUE) {
            frames++;
        }
    }
    check_eq(frames, inv_tune_count(), "last tune still sent while it is away");

    heartbeat = 1u;
    value_offset = 1u; /* Retuned while off */
    const uint32_t ms = run_until_done(10000u);
    check(ms >= INV_TUNE_SETTLE_MS, "settles again before reading");
    check_eq(inv_tune_value(0), fake_value(174u), "new value read");
    check(inv_tune_crc() != crc_before, "crc follows the tune");

    inv_tune_pack_next(&id, d);
    while (id != INV_TUNE_CAN_ID_STATUS) {
        inv_tune_pack_next(&id, d);
    }
    check_eq(d[3], 2, "two sweeps");
}

static void test_isr_tick_ahead_of_loop(void)
{
    printf("test_isr_tick_ahead_of_loop\n");

    reset();
    run_until_done(10000u);
    /* 0x0AA lands between the loop reading the tick and calling update */
    inv_tune_feed_state(now + 1u);
    uint8_t req[8];
    inv_tune_update(now, 0u, req);
    check_eq(inv_tune_state(), INV_TUNE_DONE, "not mistaken for a lost inverter");
}

static void test_response_not_overwritten(void)
{
    printf("test_response_not_overwritten\n");

    reset();
    answering = 0u;
    run_ms(INV_TUNE_SETTLE_MS + 2u * LOOP_MS);
    reply(174u, 2027u);
    reply(999u, 1u); /* Dropped, one already waiting */
    step();
    check_eq(inv_tune_value(0), 2027, "first reply kept");
    check_eq(requests, 2, "moved on");
}

static void test_frames(void)
{
    printf("test_frames\n");

    reset();
    uint16_t id;
    uint8_t d[8];
    uint8_t sent = 0u;
    for (uint8_t i = 0; i < 4u; i++) {
        memset(d, 0xEE, sizeof(d));
        const uint8_t dlc = inv_tune_pack_next(&id, d);
        if (dlc) {
            sent++;
            check_eq(id, INV_TUNE_CAN_ID_STATUS, "only status before the first sweep");
            check_eq(dlc, INV_TUNE_STATUS_DLC, "status dlc");
            check_eq(d[0], INV_TUNE_WAITING, "state");
            check_eq(d[6], inv_tune_count(), "parameter count");
        }
    }
    check_eq(sent, 2, "status every other slot");

    run_until_done(10000u);
    uint8_t seen[128] = {0};
    uint8_t alternates = 1u;
    for (uint16_t i = 0; i < 2u * inv_tune_count(); i++) {
        memset(d, 0xEE, sizeof(d));
        const uint8_t dlc = inv_tune_pack_next(&id, d);
        if ((i % 2u == 0u) != (id == INV_TUNE_CAN_ID_STATUS)) {
            alternates = 0u;
        }
        if (id == INV_TUNE_CAN_ID_STATUS) {
            check_eq(d[0], INV_TUNE_DONE, "done");
            check_eq(d[4] | d[5] << 8, inv_tune_crc(), "crc little endian");
            continue;
        }
        check_eq(id, INV_TUNE_CAN_ID_VALUE, "value id");
        check_eq(dlc, INV_TUNE_VALUE_DLC, "value dlc");
        const uint16_t addr = (uint16_t)(d[0] | d[1] << 8);
        const uint16_t value = (uint16_t)(d[2] | d[3] << 8);
        check_eq(value, fake_value(addr), "value matches its address, little endian");
        for (uint8_t p = 0; p < inv_tune_count(); p++) {
            if (inv_tune_addrs[p] == addr) {
                check_eq(p, (i - 1u) / 2u, "table order");
                seen[p]++;
            }
        }
    }
    check(alternates, "status and value take turns");
    uint8_t each_once = 1u;
    for (uint8_t p = 0; p < inv_tune_count(); p++) {
        if (seen[p] != 1u) {
            each_once = 0u;
        }
    }
    check(each_once, "every parameter once per rotation");
}

static void test_signed_value(void)
{
    printf("test_signed_value\n");

    reset();
    answering = 0u;
    run_ms(INV_TUNE_SETTLE_MS + 2u * LOOP_MS);
    for (uint8_t i = 0; i < 9u; i++) {
        reply(inv_tune_addrs[i], 1u);
        step();
    }
    reply(152u, (uint16_t)-1050);
    step();
    check_eq((int16_t)inv_tune_value(9), -1050, "gamma adjust keeps its sign bits");
}

static void test_crc(void)
{
    printf("test_crc\n");

    const uint8_t s[] = "123456789";
    check_eq(crc16(0xFFFFu, s, 9u), 0x29B1, "CRC-16/CCITT-FALSE check value");
}

int main(void)
{
    test_table();
    test_waits_for_inverter();
    test_full_sweep();
    test_ignores_other_addresses();
    test_retries_then_missing();
    test_pauses_while_enabled();
    test_rereads_after_inverter_restart();
    test_isr_tick_ahead_of_loop();
    test_response_not_overwritten();
    test_frames();
    test_signed_value();
    test_crc();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
