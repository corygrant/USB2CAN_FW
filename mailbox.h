#pragma once

#include <cstdint>
#include "hal.h"

#define MAILBOX_SIZE 64

// CAN
msg_t PostCanTxFrame(CANTxFrame *frame);
msg_t FetchCanTxFrame(CANTxFrame *frame);

msg_t PostCanRxFrame(CANRxFrame *frame);
msg_t FetchCanRxFrame(CANRxFrame *frame);

// USB
msg_t PostUsbTxFrame(CANTxFrame *frame);
msg_t FetchUsbTxFrame(CANTxFrame *frame);

msg_t PostUsbRxFrame(CANRxFrame *frame);
msg_t FetchUsbRxFrame(CANRxFrame *frame);