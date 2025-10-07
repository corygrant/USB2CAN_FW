#pragma once

#include "hal.h"
#include "enums.h"

#define SYS_TIME TIME_I2MS(chVTGetSystemTimeX())

CANConfig GetCanConfig(CanBitrate bitrate, CanMode eMode);