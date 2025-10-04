#include "ch.hpp"
#include "hal.h"

#include "usb2can_config.h"
#include "can.h"
#include "usb.h"
#include "enums.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  //Set the bitrate here
  InitCan(CanBitrate::Bitrate_500K);

  InitUsb();

  while (true)
  {
    // FetchUSBRXFrame
    // Check for SLCAN commands and handle them
    // Or PostCanTxFrame

    // FetchCanRxFrame
    // PostUsbTxFrame


    chThdSleepMilliseconds(500);
  }
}
