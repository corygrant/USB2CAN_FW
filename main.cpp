#include "ch.h"
#include "hal.h"

#include "usb2can_config.h"
#include "can.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  InitCan();

  while (true)
  {
    UpdateSwPos();
    chThdSleepMilliseconds(500);
  }
}
