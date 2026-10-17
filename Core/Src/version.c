#include "version.h"

/* Written by tools/version.sh before each build */
#if __has_include("build_version.h")
#include "build_version.h"
#else
#define BUILD_HASH  0u
#define BUILD_FLAGS VERSION_FLAG_NONE
#endif

void version_pack(uint8_t *d)
{
    const uint16_t hi = (uint16_t)((uint32_t)BUILD_HASH >> 16);
    const uint16_t lo = (uint16_t)((uint32_t)BUILD_HASH & 0xFFFF);

    d[0] = (uint8_t)(hi & 0xFF);
    d[1] = (uint8_t)(hi >> 8);
    d[2] = (uint8_t)(lo & 0xFF);
    d[3] = (uint8_t)(lo >> 8);
    d[4] = BUILD_FLAGS;
}

void version_tx_init(version_tx_t *v, uint32_t now_ms)
{
    /* Backdated so the first frame goes out right away */
    v->last_ms = now_ms - VERSION_TX_MS;
    v->sent = 0u;
}

uint8_t version_tx_due(version_tx_t *v, uint32_t now_ms)
{
    const uint32_t period = (v->sent < VERSION_TX_COUNT) ? VERSION_TX_MS : VERSION_REPEAT_MS;

    if (now_ms - v->last_ms < period) {
        return 0u;
    }
    v->last_ms = now_ms;
    /* Stops counting once past the burst, so it can't wrap back into it */
    if (v->sent < VERSION_TX_COUNT) {
        v->sent++;
    }
    return 1u;
}
