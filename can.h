#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"

msg_t InitCan(CanBitrate eBitrate);
void StopCan();