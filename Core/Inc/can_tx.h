/**
 * @file    can_tx.h
 * @brief   Software transmit queue for the CAN peripheral.
 *
 * Callers enqueue and never block. can_tx_pump() moves frames into free
 * hardware mailboxes. A full queue drops the newest frame and counts it.
 *
 * Not ISR safe: enqueue and pump both run in main loop context.
 */

#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdint.h>

#define CAN_TX_QUEUE_LEN 16

void can_tx_init(void);

/* Queue one standard-identifier data frame. Bytes past dlc are zero filled.
 * Returns 0 if the queue was full and the frame was dropped. */
uint8_t can_tx_send(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* Move queued frames into free mailboxes. Once per loop pass. */
void can_tx_pump(void);

uint16_t can_tx_dropped(void);
uint8_t can_tx_depth(void);

#endif /* CAN_TX_H */
