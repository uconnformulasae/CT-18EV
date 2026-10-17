#ifndef VERSION_H
#define VERSION_H

#include <stdint.h>

#define VERSION_CAN_ID 0x55B
#define VERSION_DLC    5
#define VERSION_TX_MS  1000u

/* Sent this many times after boot, then never */
#define VERSION_TX_COUNT 10u

#define VERSION_FLAG_SNAPSHOT 0x01 /* Built with uncommitted changes */
#define VERSION_FLAG_NONE     0x02 /* No git info at build time */

typedef struct {
    uint32_t last_ms;
    uint8_t sent;
} version_tx_t;

void version_pack(uint8_t *d);

void version_tx_init(version_tx_t *v, uint32_t now_ms);

/* 1 when a frame should go out now; counts it as sent */
uint8_t version_tx_due(version_tx_t *v, uint32_t now_ms);

#endif /* VERSION_H */
