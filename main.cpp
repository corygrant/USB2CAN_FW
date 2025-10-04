#include "ch.hpp"
#include "hal.h"

#include "usb2can_config.h"
#include "can.h"
#include "enums.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  InitCan(CanBitrate::Bitrate_500K);

  while (true)
  {
    chThdSleepMilliseconds(500);
  }
}
