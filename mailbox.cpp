#include "mailbox.h"
#include "ch.hpp"

static chibios_rt::Mailbox<CANRxFrame*, MAILBOX_SIZE> rxCanMb;
static chibios_rt::Mailbox<CANTxFrame*, MAILBOX_SIZE> txCanMb;
static chibios_rt::Mailbox<CANRxFrame*, MAILBOX_SIZE> rxUsbMb;
static chibios_rt::Mailbox<CANTxFrame*, MAILBOX_SIZE> txUsbMb;

//Mailbox buffer of CAN frames
//Not managed by mailbox
CANRxFrame rxCanFrames[MAILBOX_SIZE];
CANTxFrame txCanFrames[MAILBOX_SIZE];
CANRxFrame rxUsbFrames[MAILBOX_SIZE];
CANTxFrame txUsbFrames[MAILBOX_SIZE];

//Used to manage the memory used by the mailbox
bool rxCanMsgUsed[MAILBOX_SIZE];
bool txCanMsgUsed[MAILBOX_SIZE];
bool rxUsbMsgUsed[MAILBOX_SIZE];
bool txUsbMsgUsed[MAILBOX_SIZE];

msg_t PostCanTxFrame(CANTxFrame *frame)
{
    for (int i = 0; i < MAILBOX_SIZE; i++) {
        if (!txCanMsgUsed[i]) {
            // Find a free slot in can_messages[] to store the data
            txCanFrames[i] = *frame;
            txCanMsgUsed[i] = true;

            // Try to post the pointer to the mailbox
            msg_t result = txCanMb.post(&txCanFrames[i], TIME_IMMEDIATE);
            if (result == MSG_OK) {
                return result;  // Successfully posted
            } else {
                txCanMsgUsed[i] = false;  // Free the slot if mailbox is full
                return result;
            }
        }
    }

    return MSG_TIMEOUT;  // No free slots
}

msg_t FetchCanTxFrame(CANTxFrame *frame)
{
    CANTxFrame *txFrame;
    // Fetch a pointer from the mailbox
    msg_t result = txCanMb.fetch(&txFrame, TIME_IMMEDIATE);
    if (result == MSG_OK) {
        // Mark the slot in memory as free
        for (int i = 0; i < MAILBOX_SIZE; i++) {
            if (txFrame == &txCanFrames[i]) {
                txCanMsgUsed[i] = false;
                break;
            }
        }
        *frame = *txFrame;
        return result;
    }
    return result;
}

msg_t PostCanRxFrame(CANRxFrame *frame)
{
    for (int i = 0; i < MAILBOX_SIZE; i++) {
        if (!rxCanMsgUsed[i]) {
            // Find a free slot in can_messages[] to store the data
            rxCanFrames[i] = *frame;
            rxCanMsgUsed[i] = true;

            // Try to post the pointer to the mailbox
            msg_t result = rxCanMb.post(&rxCanFrames[i], TIME_IMMEDIATE);
            if (result == MSG_OK) {
                return result;  // Successfully posted
            } else {
                rxCanMsgUsed[i] = false;  // Free the slot if mailbox is full
                return result;
            }
        }
    }

    return MSG_TIMEOUT;  // No free slots
}

msg_t FetchCanRxFrame(CANRxFrame *frame)
{
    CANRxFrame *rxFrame;
    // Fetch a pointer from the mailbox
    msg_t result = rxCanMb.fetch(&rxFrame, TIME_IMMEDIATE);
    if (result == MSG_OK) {
        // Mark the slot in memory as free
        for (int i = 0; i < MAILBOX_SIZE; i++) {
            if (rxFrame == &rxCanFrames[i]) {
                rxCanMsgUsed[i] = false;
                break;
            }
        }
        *frame = *rxFrame;
        return result;
    }
    return result;
}

msg_t PostUsbTxFrame(CANTxFrame *frame)
{
    for (int i = 0; i < MAILBOX_SIZE; i++) {
        if (!txUsbMsgUsed[i]) {
            // Find a free slot in can_messages[] to store the data
            txUsbFrames[i] = *frame;
            txUsbMsgUsed[i] = true;

            // Try to post the pointer to the mailbox
            msg_t result = txUsbMb.post(&txUsbFrames[i], TIME_IMMEDIATE);
            if (result == MSG_OK) {
                return result;  // Successfully posted
            } else {
                txUsbMsgUsed[i] = false;  // Free the slot if mailbox is full
                return result;
            }
        }
    }

    return MSG_TIMEOUT;  // No free slots
}

msg_t FetchUsbTxFrame(CANTxFrame *frame)
{
    CANTxFrame *txFrame;
    // Fetch a pointer from the mailbox
    msg_t result = txUsbMb.fetch(&txFrame, TIME_IMMEDIATE);
    if (result == MSG_OK) {
        // Mark the slot in memory as free
        for (int i = 0; i < MAILBOX_SIZE; i++) {
            if (txFrame == &txUsbFrames[i]) {
                txUsbMsgUsed[i] = false;
                break;
            }
        }
        *frame = *txFrame;
        return result;
    }
    return result;
}

msg_t PostUsbRxFrame(CANRxFrame *frame)
{
    for (int i = 0; i < MAILBOX_SIZE; i++) {
        if (!rxUsbMsgUsed[i]) {
            // Find a free slot in can_messages[] to store the data
            rxUsbFrames[i] = *frame;
            rxUsbMsgUsed[i] = true;

            // Try to post the pointer to the mailbox
            msg_t result = rxUsbMb.post(&rxUsbFrames[i], TIME_IMMEDIATE);
            if (result == MSG_OK) {
                return result;  // Successfully posted
            } else {
                rxUsbMsgUsed[i] = false;  // Free the slot if mailbox is full
                return result;
            }
        }
    }

    return MSG_TIMEOUT;  // No free slots
}

msg_t FetchUsbRxFrame(CANRxFrame *frame)
{
    CANRxFrame *rxFrame;
    // Fetch a pointer from the mailbox
    msg_t result = rxUsbMb.fetch(&rxFrame, TIME_IMMEDIATE);
    if (result == MSG_OK) {
        // Mark the slot in memory as free
        for (int i = 0; i < MAILBOX_SIZE; i++) {
            if (rxFrame == &rxUsbFrames[i]) {
                rxUsbMsgUsed[i] = false;
                break;
            }
        }
        *frame = *rxFrame;
        return result;
    }
    return result;
}