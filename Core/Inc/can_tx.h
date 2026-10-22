#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdint.h>

#define CAN_TX_QUEUE_LEN 16

void can_tx_init(void);

/* Queue standard CAN frame. Returns 0 if dropped. */
uint8_t can_tx_send(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/*
 * Inverter command. Skips the queue and always finds a mailbox, since the
 * queue never fills the last one. A newer command replaces one not yet sent.
 */
void can_tx_send_command(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* Drain queue into free hardware mailboxes */
void can_tx_pump(void);

uint16_t can_tx_dropped(void);
uint8_t can_tx_depth(void);
uint16_t can_tx_cmd_replaced(void); /* Commands dropped as stale */

#endif /* CAN_TX_H */
