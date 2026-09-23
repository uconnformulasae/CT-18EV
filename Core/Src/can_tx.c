/* Software CAN transmit queue. Builds for the host tests with
 * -DCAN_TX_HOST. */

#include "can_tx.h"

typedef struct {
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_tx_frame_t;

#ifdef CAN_TX_HOST

/* Host build: fake mailboxes the tests drive directly. */
uint32_t can_tx_host_free_mailboxes = 3u;
can_tx_frame_t can_tx_host_sent[64];
uint32_t can_tx_host_sent_count = 0u;

static uint32_t mailboxes_free(void)
{
    return can_tx_host_free_mailboxes;
}

static uint8_t mailbox_send(const can_tx_frame_t *f)
{
    if (can_tx_host_free_mailboxes == 0u) {
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

    /* Built per send so every field is set; the HAL reads TransmitGlobalTime. */
    header.StdId = f->std_id;
    header.ExtId = 0u;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = f->dlc;
    header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)f->data, &mailbox) == HAL_OK) ? 1u : 0u;
}

#endif

static can_tx_frame_t queue[CAN_TX_QUEUE_LEN];
static uint8_t q_head; /* next write */
static uint8_t q_tail; /* next read  */
static uint8_t q_count;
static uint16_t q_dropped;

void can_tx_init(void)
{
    q_head = 0u;
    q_tail = 0u;
    q_count = 0u;
    q_dropped = 0u;
}

uint8_t can_tx_send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (q_count >= CAN_TX_QUEUE_LEN) {
        q_dropped++;
        return 0u;
    }

    can_tx_frame_t *f = &queue[q_head];
    f->std_id = std_id;
    f->dlc = (dlc > 8u) ? 8u : dlc;
    for (uint8_t i = 0u; i < 8u; i++) {
        f->data[i] = (i < f->dlc) ? data[i] : 0u;
    }

    q_head = (uint8_t)((q_head + 1u) % CAN_TX_QUEUE_LEN);
    q_count++;
    return 1u;
}

void can_tx_pump(void)
{
    while (q_count > 0u && mailboxes_free() > 0u) {
        if (!mailbox_send(&queue[q_tail])) {
            break; /* refused despite reporting free; retry next pass */
        }
        q_tail = (uint8_t)((q_tail + 1u) % CAN_TX_QUEUE_LEN);
        q_count--;
    }
}

uint16_t can_tx_dropped(void)
{
    return q_dropped;
}

uint8_t can_tx_depth(void)
{
    return q_count;
}
