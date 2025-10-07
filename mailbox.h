#pragma once

#include <cstdint>
#include "hal.h"
#include "enums.h"

#define MAILBOX_SIZE 64

// CAN
msg_t PostCanTxFrame(CANTxFrame *frame);
msg_t FetchCanTxFrame(CANTxFrame *frame);

//CanRx frame goes to UsbTx

// USB
msg_t PostUsbTxFrame(CANTxFrame *frame);
msg_t FetchUsbTxFrame(CANTxFrame *frame);

msg_t PostUsbRxFrame(SlcanRxFrame *frame);
msg_t FetchUsbRxFrame(SlcanRxFrame *frame);

void ClearMailboxes();