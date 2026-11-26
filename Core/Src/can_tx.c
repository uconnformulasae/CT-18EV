/* CAN transmit queue */

#include "can_tx.h"

typedef struct {
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_tx_frame_t;

#ifdef CAN_TX_HOST

uint32_t can_tx_host_free_mailboxes = 3u;
uint8_t can_tx_host_refuse = 0u; /* AddTxMessage failing with a mailbox free */
can_tx_frame_t can_tx_host_sent[64];
uint32_t can_tx_host_sent_count = 0u;

static uint32_t mailboxes_free(void)
{
    return can_tx_host_free_mailboxes;
}

/* Only called once mailboxes_free() says there's room */
static uint8_t mailbox_send(const can_tx_frame_t *f)
{
    if (can_tx_host_refuse) {
        return 0u;
    }
    if (can_tx_host_sent_count < 64u) {
        can_tx_host_sent[can_tx_host_sent_count++] = *f;
    }
    can_tx_host_free_mailboxes--;
    return 1u;
}

#else

#include "main.h"

extern CAN_HandleTypeDef hcan;

static uint32_t mailboxes_free(void)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
}

static uint8_t mailbox_send(const can_tx_frame_t *f)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;

    header.StdId = f->std_id;
    header.ExtId = 0u;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = f->dlc;
    header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)f->data, &mailbox) == HAL_OK) ? 1u : 0u;
}

#endif

can_tx_queue_t* can_tx_mk_queue() {
	can_tx_queue_t queue;
	can_tx_init(&queue);
	return queue;
}

void can_tx_init(can_tx_queue_t* self)
{
    self->q_head = 0u;
    self->q_tail = 0u;
    self->q_count = 0u;
    self->q_dropped = 0u;
}

uint8_t can_tx_send(can_tx_queue_t* self, uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (self->q_count >= CAN_TX_QUEUE_LEN) {
        self->q_dropped++;
        return 0u;
    }

    can_tx_frame_t* f = &self->queue[self->q_head];

    f->std_id = std_id;
    f->dlc = (dlc > 8u) ? 8u : dlc;

    for (uint8_t i = 0u; i < 8u; i++) {
        f->data[i] = (i < f->dlc) ? data[i] : 0u;
    }

    self->q_head = (uint8_t)((self->q_head + 1u) % CAN_TX_QUEUE_LEN);
    self->q_count++;
    return 1u;
}

void can_tx_pump(can_tx_queue_t* self)
{
    while (self->q_count > 0u && mailboxes_free() > 0u) {
        if (!mailbox_send(&self->queue[q_tail])) {
            break; /* mailbox busy */
        }

        self->q_tail = (uint8_t)((self->q_tail + 1u) % CAN_TX_QUEUE_LEN);
        self->q_count--;
    }
}

uint16_t can_tx_dropped(const can_tx_queue_t* self)
{
    return self->q_dropped;
}

uint8_t can_tx_depth(const can_tx_queue_t* self)
{
    return self->q_count;
}
