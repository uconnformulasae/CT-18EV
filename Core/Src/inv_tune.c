/* Inverter EEPROM readback over the 0x0C1/0x0C2 parameter messages */

#include "inv_tune.h"
#include "inv_tune_params.h"

#define N_PARAMS ((uint8_t)(sizeof(inv_tune_addrs) / sizeof(inv_tune_addrs[0])))

/* Written by the RX interrupt */
static volatile uint32_t last_seen;
static volatile uint8_t seen;
static volatile uint8_t resp_full; /* ISR only fills an empty slot */
static volatile uint16_t resp_addr;
static volatile uint16_t resp_value;

static uint16_t values[N_PARAMS];
static uint8_t have_tune; /* At least one sweep done */
static uint8_t state;
static uint32_t settle_start;
static uint8_t idx;
static uint8_t tries;
static uint8_t outstanding;
static uint32_t sent_tick;
static uint8_t n_ok;
static uint8_t n_missing;
static uint8_t sweeps;
static uint16_t crc;
static uint8_t rot;
static uint8_t status_next;

static uint16_t crc16(uint16_t c, const uint8_t *p, uint32_t n)
{
    while (n--) {
        c ^= (uint16_t)(*p++ << 8);
        for (uint8_t b = 0; b < 8u; b++) {
            c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
        }
    }
    return c;
}

void inv_tune_init(void)
{
    last_seen = 0u;
    seen = 0u;
    resp_full = 0u;
    for (uint8_t i = 0; i < N_PARAMS; i++) {
        values[i] = INV_TUNE_MISSING;
    }
    have_tune = 0u;
    state = INV_TUNE_WAITING;
    outstanding = 0u;
    n_ok = 0u;
    n_missing = 0u;
    sweeps = 0u;
    crc = 0u;
    rot = 0u;
    status_next = 1u;
}

void inv_tune_feed_state(uint32_t tick_ms)
{
    last_seen = tick_ms;
    seen = 1u;
}

void inv_tune_feed_response(const uint8_t *d)
{
    if (resp_full) {
        return;
    }
    resp_addr = (uint16_t)(d[1] << 8 | d[0]);
    resp_value = (uint16_t)(d[5] << 8 | d[4]);
    resp_full = 1u;
}

static void finish_sweep(void)
{
    crc = 0xFFFFu;
    for (uint8_t i = 0; i < N_PARAMS; i++) {
        const uint8_t b[2] = {(uint8_t)(values[i] & 0xFF), (uint8_t)(values[i] >> 8)};
        crc = crc16(crc, b, 2u);
    }
    have_tune = 1u;
    sweeps++;
    state = INV_TUNE_DONE;
}

static void advance(void)
{
    idx++;
    tries = 0u;
    outstanding = 0u;
    if (idx >= N_PARAMS) {
        finish_sweep();
    }
}

uint8_t inv_tune_update(uint32_t tick_ms, uint8_t inverter_enabled, uint8_t *req)
{
    /* Signed, the ISR can stamp a tick newer than ours */
    const uint8_t alive = seen && (int32_t)(tick_ms - last_seen) <= (int32_t)INV_TUNE_LOST_MS;

    if (!alive) {
        state = INV_TUNE_WAITING;
        return 0u;
    }
    if (state == INV_TUNE_WAITING) {
        state = INV_TUNE_SETTLING;
        settle_start = tick_ms;
    }
    if (state == INV_TUNE_SETTLING) {
        if (tick_ms - settle_start < INV_TUNE_SETTLE_MS) {
            return 0u;
        }
        idx = 0u;
        tries = 0u;
        outstanding = 0u;
        n_ok = 0u;
        n_missing = 0u;
        resp_full = 0u;
        state = INV_TUNE_READING;
    }
    if (state == INV_TUNE_DONE) {
        return 0u;
    }

    if (resp_full) {
        const uint16_t addr = resp_addr;
        const uint16_t value = resp_value;
        resp_full = 0u;
        /* Late or unrecognized (0,0) replies fall through to the timeout */
        if (outstanding && addr == inv_tune_addrs[idx]) {
            values[idx] = value;
            n_ok++;
            advance();
        }
    }
    if (outstanding && tick_ms - sent_tick >= INV_TUNE_REPLY_MS) {
        outstanding = 0u;
        if (++tries > INV_TUNE_RETRIES) {
            values[idx] = INV_TUNE_MISSING;
            n_missing++;
            advance();
        }
    }
    if (state == INV_TUNE_DONE || outstanding) {
        return 0u;
    }
    if (inverter_enabled) {
        state = INV_TUNE_PAUSED;
        return 0u;
    }

    state = INV_TUNE_READING;
    const uint16_t addr = inv_tune_addrs[idx];
    req[0] = (uint8_t)(addr & 0xFF);
    req[1] = (uint8_t)(addr >> 8);
    for (uint8_t i = 2; i < 8u; i++) {
        req[i] = 0u; /* Byte 2 = 0 is read, nothing here ever writes */
    }
    outstanding = 1u;
    sent_tick = tick_ms;
    return 1u;
}

uint8_t inv_tune_pack_next(uint16_t *id, uint8_t *d)
{
    /* Every other slot, keeps the status well inside the AiM timeout */
    const uint8_t status = status_next;
    status_next = !status_next;

    if (status) {
        *id = INV_TUNE_CAN_ID_STATUS;
        d[0] = state;
        d[1] = n_ok;
        d[2] = n_missing;
        d[3] = sweeps;
        d[4] = (uint8_t)(crc & 0xFF);
        d[5] = (uint8_t)(crc >> 8);
        d[6] = N_PARAMS;
        return INV_TUNE_STATUS_DLC;
    }
    if (!have_tune) {
        return 0u;
    }

    const uint16_t addr = inv_tune_addrs[rot];
    *id = INV_TUNE_CAN_ID_VALUE;
    d[0] = (uint8_t)(addr & 0xFF);
    d[1] = (uint8_t)(addr >> 8);
    d[2] = (uint8_t)(values[rot] & 0xFF);
    d[3] = (uint8_t)(values[rot] >> 8);
    rot = (uint8_t)((rot + 1u) % N_PARAMS);
    return INV_TUNE_VALUE_DLC;
}

uint8_t inv_tune_state(void)
{
    return state;
}

uint8_t inv_tune_count(void)
{
    return N_PARAMS;
}

uint16_t inv_tune_value(uint8_t i)
{
    return (i < N_PARAMS) ? values[i] : INV_TUNE_MISSING;
}

uint16_t inv_tune_crc(void)
{
    return crc;
}
