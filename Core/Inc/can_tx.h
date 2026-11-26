#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdint.h>

#define CAN_TX_QUEUE_LEN 16

#ifdef CAN_TX_HOST
#endif

typedef struct can_tx_frame {
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_tx_frame_t;

typedef struct can_tx_queue {
	can_tx_frame_t queue[CAN_TX_QUEUE_LEN];
	uint8_t q_head;
	uint8_t q_tail;
	uint8_t q_count;
	uint16_t q_dropped;
} can_tx_queue_t;

typedef struct can_tx_host {
	uint32_t can_tx_host_free_mailboxes;
	uint8_t can_tx_host_refuse;
	can_tx_frame_t can_tx_host_sent[64];
	uint32_t can_tx_host_sent_count;
} can_tx_host_t;

can_tx_queue_t* can_tx_mk_queue();
void can_tx_init(can_tx_queue_t* self);

/* Queue standard CAN frame. Returns 0 if dropped. */
uint8_t can_tx_send(can_tx_queue_t* self, uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* Drain queue into free hardware mailboxes */
void can_tx_pump(can_tx_queue_t* self);

uint16_t can_tx_dropped(const can_tx_queue_t* self);
uint8_t can_tx_depth(const can_tx_queue_t* self);



#endif /* CAN_TX_H */
