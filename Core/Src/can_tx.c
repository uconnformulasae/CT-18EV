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
uint32_t can_tx_host_pending = 0u; /* Mailbox bits not yet on the bus */
uint32_t can_tx_host_aborts = 0u;

static uint32_t mailboxes_free(void)
{
    return can_tx_host_free_mailboxes;
}

/* Only called once mailboxes_free() says there's room */
static uint8_t mailbox_send(const can_tx_frame_t *f, uint32_t *mailbox)
{
    if (can_tx_host_refuse) {
        return 0u;
    }
    if (can_tx_host_sent_count < 64u) {
        can_tx_host_sent[can_tx_host_sent_count++] = *f;
    }
    can_tx_host_free_mailboxes--;
    *mailbox = 0u; /* Lowest free, like the hardware; 0 once tests overfill */
    for (uint32_t b = 1u; b <= 4u; b <<= 1) {
        if (!(can_tx_host_pending & b)) {
            *mailbox = b;
            break;
        }
    }
    can_tx_host_pending |= *mailbox;
    return 1u;
}

static uint8_t mailbox_pending(uint32_t mailbox)
{
    return (can_tx_host_pending & mailbox) ? 1u : 0u;
}

static void mailbox_abort(uint32_t mailbox)
{
    can_tx_host_pending &= ~mailbox;
    can_tx_host_free_mailboxes++;
    can_tx_host_aborts++;
}

#else

#include "main.h"

extern CAN_HandleTypeDef hcan;

static uint32_t mailboxes_free(void)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
}

static uint8_t mailbox_send(const can_tx_frame_t *f, uint32_t *mailbox)
{
    CAN_TxHeaderTypeDef header;

    header.StdId = f->std_id;
    header.ExtId = 0u;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = f->dlc;
    header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)f->data, mailbox) == HAL_OK) ? 1u : 0u;
}

static uint8_t mailbox_pending(uint32_t mailbox)
{
    return HAL_CAN_IsTxMessagePending(&hcan, mailbox) ? 1u : 0u;
}

/* Frees the mailbox, unless the frame is already on the wire and goes through */
static void mailbox_abort(uint32_t mailbox)
{
    HAL_CAN_AbortTxRequest(&hcan, mailbox);
}

#endif

static can_tx_frame_t queue[CAN_TX_QUEUE_LEN];
static uint8_t q_head;
static uint8_t q_tail;
static uint8_t q_count;
static uint16_t q_dropped;

static can_tx_frame_t cmd;
static uint8_t cmd_waiting;  /* Not in a mailbox yet */
static uint32_t cmd_mailbox; /* Last command's mailbox, 0 once done with */
static uint16_t cmd_replaced;

static void fill(can_tx_frame_t *f, uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    f->std_id = std_id;
    f->dlc = (dlc > 8u) ? 8u : dlc;
    for (uint8_t i = 0u; i < 8u; i++) {
        f->data[i] = (i < f->dlc) ? data[i] : 0u;
    }
}

void can_tx_init(void)
{
    q_head = 0u;
    q_tail = 0u;
    q_count = 0u;
    q_dropped = 0u;
    cmd_waiting = 0u;
    cmd_mailbox = 0u;
    cmd_replaced = 0u;
}

uint8_t can_tx_send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (q_count >= CAN_TX_QUEUE_LEN) {
        q_dropped++;
        return 0u;
    }

    fill(&queue[q_head], std_id, data, dlc);
    q_head = (uint8_t)((q_head + 1u) % CAN_TX_QUEUE_LEN);
    q_count++;
    return 1u;
}

void can_tx_send_command(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    /* Still waiting a whole loop later means it is stale, send the new one */
    if (cmd_waiting) {
        cmd_replaced++;
    } else if (cmd_mailbox && mailbox_pending(cmd_mailbox)) {
        mailbox_abort(cmd_mailbox);
        cmd_replaced++;
    }
    cmd_mailbox = 0u;
    fill(&cmd, std_id, data, dlc);
    cmd_waiting = 1u;
    can_tx_pump();
}

void can_tx_pump(void)
{
    if (cmd_waiting && mailboxes_free() > 0u && mailbox_send(&cmd, &cmd_mailbox)) {
        cmd_waiting = 0u;
    }

    /* The last free mailbox is kept for the next command */
    while (q_count > 0u && mailboxes_free() > 1u) {
        uint32_t mailbox;
        if (!mailbox_send(&queue[q_tail], &mailbox)) {
            break; /* mailbox busy */
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

uint16_t can_tx_cmd_replaced(void)
{
    return cmd_replaced;
}
