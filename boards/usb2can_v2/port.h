#pragma once

#include "hal.h"
#include "enums.h"

#define SYS_TIME TIME_I2MS(chVTGetSystemTimeX())

const CANConfig& GetCanConfig(CanBitrate bitrate);