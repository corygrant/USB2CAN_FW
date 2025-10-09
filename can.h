#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"

msg_t InitCan(CanBitrate eBitrate, CanMode eMode, bool bAutoRetry);
void StopCan();
CANTxFrame RxToTxFrame(CANRxFrame *stRxFrame);