#ifndef VERSION_H
#define VERSION_H

#include <stdint.h>

#define VERSION_CAN_ID 0x55B
#define VERSION_DLC    5
#define VERSION_TX_MS  500u /* AiM shows dashes if this is too slow */

#define VERSION_FLAG_SNAPSHOT 0x01 /* Built with uncommitted changes */
#define VERSION_FLAG_NONE     0x02 /* No git info at build time */

void version_pack(uint8_t *d);

#endif /* VERSION_H */
