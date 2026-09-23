/* Host tests for can_tx */


#include <stdio.h>
#include <string.h>

#include "../Core/Src/can_tx.c"

static int failures = 0;
static int checks = 0;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_eq(long got, long want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL: %s (got %ld want %ld)\n", what, got, want);
    }
}

/* Reset the module and the fake mailboxes. */
static void reset_all(uint32_t free_mailboxes)
{
    can_tx_init();
    can_tx_host_free_mailboxes = free_mailboxes;
    can_tx_host_sent_count = 0u;
    memset(can_tx_host_sent, 0, sizeof(can_tx_host_sent));
}

static const uint8_t PAYLOAD[8] = {1, 2, 3, 4, 5, 6, 7, 8};

static void test_enqueue_and_pump(void)
{
    printf("test_enqueue_and_pump\n");

    reset_all(3u);
    check_eq(can_tx_depth(), 0, "starts empty");
    check_eq(can_tx_dropped(), 0, "starts with no drops");

    check_eq(can_tx_send(0x0C0, PAYLOAD, 8), 1, "send accepts a frame");
    check_eq(can_tx_depth(), 1, "depth tracks the queued frame");
    check_eq(can_tx_host_sent_count, 0, "nothing transmits until the pump runs");

    can_tx_pump();
    check_eq(can_tx_depth(), 0, "pump drains the queue");
    check_eq(can_tx_host_sent_count, 1, "one frame reached a mailbox");
    check_eq(can_tx_host_sent[0].std_id, 0x0C0, "identifier carried through");
    check_eq(can_tx_host_sent[0].dlc, 8, "dlc carried through");
    check_eq(memcmp(can_tx_host_sent[0].data, PAYLOAD, 8), 0, "payload carried through");
}

static void test_fifo_order(void)
{
    printf("test_fifo_order\n");

    reset_all(16u);
    for (uint8_t i = 0; i < 5u; i++) {
        const uint8_t d[8] = {i};
        can_tx_send((uint16_t)(0x100 + i), d, 8);
    }
    can_tx_pump();

    check_eq(can_tx_host_sent_count, 5, "all five transmitted");
    for (uint8_t i = 0; i < 5u; i++) {
        check_eq(can_tx_host_sent[i].std_id, 0x100 + i, "frames leave in order queued");
    }
}

static void test_short_dlc_is_zero_filled(void)
{
    printf("test_short_dlc_is_zero_filled\n");

    /* RTD frame uses 1 byte from 8 byte buffer */
    reset_all(3u);
    can_tx_send(0x556, PAYLOAD, 1);
    can_tx_pump();

    check_eq(can_tx_host_sent[0].dlc, 1, "dlc preserved");
    check_eq(can_tx_host_sent[0].data[0], 1, "first byte preserved");
    for (int i = 1; i < 8; i++) {
        check_eq(can_tx_host_sent[0].data[i], 0, "trailing bytes zero filled");
    }

    /* Oversized DLC clamped to 8 */
    reset_all(3u);
    can_tx_send(0x555, PAYLOAD, 99);
    can_tx_pump();
    check_eq(can_tx_host_sent[0].dlc, 8, "dlc clamped to 8");
}

static void test_backpressure(void)
{
    printf("test_backpressure\n");

    /* Queue when mailboxes are full */
    reset_all(0u);
    for (int i = 0; i < 4; i++) {
        check_eq(can_tx_send(0x0C0, PAYLOAD, 8), 1, "queued while mailboxes are full");
    }
    can_tx_pump();
    check_eq(can_tx_host_sent_count, 0, "nothing transmits with no free mailbox");
    check_eq(can_tx_depth(), 4, "frames stay queued");
    check_eq(can_tx_dropped(), 0, "no drops yet");

    /* One mailbox frees */
    can_tx_host_free_mailboxes = 1u;
    can_tx_pump();
    check_eq(can_tx_host_sent_count, 1, "one frame leaves per free mailbox");
    check_eq(can_tx_depth(), 3, "the rest stay queued");

    /* Clear backlog */
    can_tx_host_free_mailboxes = 8u;
    can_tx_pump();
    check_eq(can_tx_depth(), 0, "backlog clears");
    check_eq(can_tx_host_sent_count, 4, "every queued frame transmitted");
}

static void test_overflow_drops_and_counts(void)
{
    printf("test_overflow_drops_and_counts\n");

    reset_all(0u);
    for (int i = 0; i < CAN_TX_QUEUE_LEN; i++) {
        check_eq(can_tx_send(0x0C0, PAYLOAD, 8), 1, "fills to capacity");
    }
    check_eq(can_tx_depth(), CAN_TX_QUEUE_LEN, "queue is full");

    /* Drop newest on overflow */
    check_eq(can_tx_send(0x0C0, PAYLOAD, 8), 0, "send reports the drop");
    check_eq(can_tx_dropped(), 1, "drop counted");
    check_eq(can_tx_depth(), CAN_TX_QUEUE_LEN, "depth unchanged by a drop");

    for (int i = 0; i < 5; i++) {
        can_tx_send(0x0C0, PAYLOAD, 8);
    }
    check_eq(can_tx_dropped(), 6, "every drop counted");

    /* Queued frames before overflow remain intact */
    can_tx_host_free_mailboxes = 64u;
    can_tx_pump();
    check_eq(can_tx_host_sent_count, CAN_TX_QUEUE_LEN, "the queued frames survived");
    check_eq(can_tx_depth(), 0, "queue empty after draining");
}

static void test_index_wraparound(void)
{
    printf("test_index_wraparound\n");

    /* Drain as we go to wrap head/tail */
    reset_all(1u);
    const int rounds = CAN_TX_QUEUE_LEN * 7 + 3;
    int transmitted = 0;

    for (int i = 0; i < rounds; i++) {
        const uint8_t d[8] = {(uint8_t)i, (uint8_t)(i >> 8)};
        check_eq(can_tx_send((uint16_t)(0x200 + (i % 16)), d, 8), 1, "queued during wraparound");
        can_tx_host_free_mailboxes = 1u;
        can_tx_host_sent_count = 0u;
        can_tx_pump();
        if (can_tx_host_sent_count == 1u) {
            check_eq(can_tx_host_sent[0].data[0], (uint8_t)transmitted,
                     "frames come out in the order they went in");
            check_eq(can_tx_host_sent[0].std_id, 0x200 + (transmitted % 16),
                     "identifier survives wraparound");
            transmitted++;
        }
    }

    check_eq(transmitted, rounds, "every frame transmitted exactly once");
    check_eq(can_tx_dropped(), 0, "no drops when the queue keeps draining");
    check_eq(can_tx_depth(), 0, "queue ends empty");
}

static void test_pump_stops_on_refusal(void)
{
    printf("test_pump_stops_on_refusal\n");

    /* Retry later without losing frame if mailbox refuses */
    reset_all(3u);
    can_tx_send(0x0C0, PAYLOAD, 8);
    can_tx_send(0x555, PAYLOAD, 8);

    can_tx_host_free_mailboxes = 0u;
    can_tx_pump();
    check_eq(can_tx_depth(), 2, "nothing lost when the mailbox refuses");

    can_tx_host_free_mailboxes = 2u;
    can_tx_pump();
    check_eq(can_tx_depth(), 0, "drains once mailboxes really are free");
    check_eq(can_tx_host_sent[0].std_id, 0x0C0, "order preserved across the refusal");
    check_eq(can_tx_host_sent[1].std_id, 0x555, "order preserved across the refusal");
}

static void test_init_clears_state(void)
{
    printf("test_init_clears_state\n");

    reset_all(0u);
    for (int i = 0; i < CAN_TX_QUEUE_LEN + 3; i++) {
        can_tx_send(0x0C0, PAYLOAD, 8);
    }
    check(can_tx_depth() > 0, "queue populated");
    check(can_tx_dropped() > 0, "drops recorded");

    can_tx_init();
    check_eq(can_tx_depth(), 0, "init empties the queue");
    check_eq(can_tx_dropped(), 0, "init clears the drop counter");
}

int main(void)
{
    test_enqueue_and_pump();
    test_fifo_order();
    test_short_dlc_is_zero_filled();
    test_backpressure();
    test_overflow_drops_and_counts();
    test_index_wraparound();
    test_pump_stops_on_refusal();
    test_init_clears_state();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
