#ifndef VERSION_H
#define VERSION_H

#include <stdint.h>

#define VERSION_CAN_ID 0x55B
#define VERSION_DLC    5
#define VERSION_TX_MS  1000u

#define VERSION_FLAG_SNAPSHOT 0x01 /* Uncommitted code, see refs/flash */
#define VERSION_FLAG_NONE     0x02 /* No git info at build time */

void version_pack(uint8_t *d);

#endif /* VERSION_H */
