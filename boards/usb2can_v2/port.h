#pragma once

#include "hal.h"
#include "led.h"
#include "enums.h"

#define SYS_TIME TIME_I2MS(chVTGetSystemTimeX())

CANConfig GetCanConfig(CanBitrate bitrate, CanMode eMode, bool bAutoRetry);