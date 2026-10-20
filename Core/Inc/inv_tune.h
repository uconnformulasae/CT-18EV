#ifndef INV_TUNE_H
#define INV_TUNE_H

#include <stdint.h>

/* Inverter EEPROM readback, rebroadcast for the AiM */

#define INV_TUNE_CAN_ID_STATUS 0x55D
#define INV_TUNE_CAN_ID_BASE   0x560 /* Tune frames, 4 parameters each */
#define INV_TUNE_STATUS_DLC    7

#define INV_TUNE_SETTLE_MS 1000u /* Let the inverter finish booting before reading */
#define INV_TUNE_LOST_MS   1000u /* 0x0AA silence that counts as an inverter restart */
#define INV_TUNE_REPLY_MS  50u
#define INV_TUNE_RETRIES   3u
#define INV_TUNE_TX_MS     16u /* Per frame, full rotation must stay under 500 ms for the AiM */

#define INV_TUNE_MISSING 0x8000u /* Sent for a parameter that never answered */

/* Status frame byte 0 */
#define INV_TUNE_WAITING  0 /* No inverter */
#define INV_TUNE_SETTLING 1
#define INV_TUNE_READING  2
#define INV_TUNE_PAUSED   3 /* Inverter enabled, reads held off */
#define INV_TUNE_DONE     4

void inv_tune_init(void);

/* Called from the CAN RX interrupt */
void inv_tune_feed_state(uint32_t tick_ms);    /* Any 0x0AA */
void inv_tune_feed_response(const uint8_t *d); /* 0x0C2 */

/* Steps the read. Returns 1 with a 0x0C1 read request in req[8] to send. */
uint8_t inv_tune_update(uint32_t tick_ms, uint8_t inverter_enabled, uint8_t *req);

/* Next frame in the AiM rotation. Returns its DLC, or 0 if the slot has nothing yet. */
uint8_t inv_tune_pack_next(uint16_t *id, uint8_t *d);

uint8_t inv_tune_state(void);
uint8_t inv_tune_count(void);
uint8_t inv_tune_frames(void);
uint16_t inv_tune_value(uint8_t i);
uint16_t inv_tune_crc(void);

#endif /* INV_TUNE_H */
